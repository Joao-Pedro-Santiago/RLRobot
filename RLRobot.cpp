#include "RLRobot.h"


RLRobot::RLRobot(ConfigFile *confFile) : Robot(confFile)
{
    cf = confFile;
    currentAction = 1; // Initialize the current action to a default state.
}

void RLRobot::init(int id) {
    Robot::init(id);
    m_state = GOING;
    pos->SetColor(RLGOING_COLOR); // Define a cor inicial no Stage

    skipCounter = 0; // Inicializa o contador de decisões
    persistence = 15; // Define quantas iterações o robô deve persistir na mesma ação antes de consultar o agente de RL novamente
    double angleDegree = 0.0;
    minDotProductToFollow = cos(angleDegree * M_PI / 180.0)* Ka; // Converte o ângulo para radianos e calcula o cosseno

    socketConnected = false; // Inicializa a flag de conexão do socket como falsa
    client_fd = socket(AF_INET, SOCK_STREAM, 0);

    int rlPort = 5000;
    try {
        rlPort = atoi(cf->valueOf("rlPort").c_str());
    } catch (...) {
        rlPort = 5000;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(rlPort);
    inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);

    reconnect(); // Tenta reconectar caso a conexão inicial falhe

}

void RLRobot::reconnect() {
    if (socketConnected) close(client_fd);
    client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == 0) {
        socketConnected = true;
    }
}

void RLRobot::mainLoop() {
    numIterations++;
    checkStall();      // Verifica se o robô travou em algo
    getPosition();     // Atualiza m_x, m_y, m_th
    

    if (skipCounter == 0 && !targetSaved){
        initialTargetX = destinationX;
        initialTargetY = destinationY;
        targetSaved = true;
    }
    

    saveLogAndWaitIfFinished();
    updateLogWhenLeavingTarget(finished, INVALID_STATE, RLGOING_END_COLOR); // Verifica se o robô saiu da região do alvo e atualiza o log


    if (skipCounter % persistence == 0) {
        double state[STATE_SIZE];
        collectState(state);
        currentAction = communicateWithPython(state);
        skipCounter = 0;
    }
    
    skipCounter++; // Incrementa o contador de decisões

    applyAction(currentAction); // Aplica a ação recebida do agente de RL

    // Verifica se chegou no alvo e define o próximo waypoint
    setNewDestinationIfOnTarget(finished, m_state, currentWaypoint, theWorld, 
                                numIterationsReachGoal, numIterations, 
                                reachingTargetTime, pos, GOING, 
                                RLGOING_OUT_COLOR , m_x, m_y, 
                                destinationX, destinationY, m_id);

    checkTimeIsOver();
    setDestination();

    // Normaliza a força resultante e aplica as velocidades aos motores
    if(fx != 0 || fy != 0){
        setConstantModule(fx, fy, maxForce);
    } 
    setSpeeds();
}

void RLRobot::collectState(double state[STATE_SIZE]) {
    state[0] = initialTargetX - m_x;
    state[1] = initialTargetY - m_y;
    state[2] = linSpeed;
    state[3] = rotSpeed;
 
    const std::vector<meters_t>& scan = laser->GetSensors()[0].ranges;
    int samplesPerSector = scan.size() / NUM_SECTORS;
 
    for (int s = 0; s < NUM_SECTORS; s++) {
        double minDist = d; // d = distância de influência
        for (int i = 0; i < samplesPerSector; i++) {
            int idx = s * samplesPerSector + i;
            if (idx < (int)scan.size() && scan[idx] < minDist) {
                minDist = scan[idx];
            }
        }
        state[4 + s] = minDist;
    }
}
int RLRobot::communicateWithPython(double state[16]) {
    if (!socketConnected) {
        reconnect();
        if (!socketConnected) return 1; 
    }
    // Envia o estado atual para o agente de RL
    if (send(client_fd, state, sizeof(double) * 16, 0) < 0) {
        socketConnected = false;
        return 1;
    }
    // Recebe a ação selecionada pelo agente de RL
    int action;
    if (recv(client_fd, &action, sizeof(int), 0) <= 0) {
        socketConnected = false;
        return 1;
    }
    return action; // Retorna a ação recebida do agente de RL
}

void RLRobot::applyAction(int action) {
    // Este método pode ser usado para aplicar a ação recebida do agente de RL, se necessário.
    // No momento, a ação é aplicada diretamente no mainLoop, mas esta função pode ser expandida para incluir lógica adicional.

    switch (action) {
        case 0:
            fx = 0; fy = 0; // AÇÃO 0: Parar (zera as forças)
            break;

        case 1:
            setAttractiveForce(fx, fy, m_x, m_y, destinationX, destinationY); // AÇÃO 1: Ir para o alvo
            obstaclesRepulsionForces();
            break;
        case 2:
            followNeighbor(); // AÇÃO 2: Seguir um vizinho, se possível
            obstaclesRepulsionForces();
            break;
        case 3:
            // AÇÃO 3: Fuga (Força repulsiva do alvo)
            setAttractiveForce(fx, fy, m_x, m_y, destinationX, destinationY);
            fx = -fx; 
            fy = -fy; 
            obstaclesRepulsionForces(); // Adiciona repulsão de obstáculos normalmente
            break;
        default:
            setAttractiveForce(fx, fy, m_x, m_y, destinationX, destinationY);
            obstaclesRepulsionForces();
    
        }

}

void RLRobot::followNeighbor() {
    const std::vector<meters_t>& scan = laser->GetSensors()[0].ranges;
    uint32_t sample_count = scan.size();
    double GoalFx, GoalFy;
    
    // 1. Calcula a direção ideal (para o alvo)
    calculateAttractiveForce(GoalFx, GoalFy, m_x, m_y, destinationX, destinationY);
    
    double influence = d; 
    // Inicializamos com o valor mínimo para aceitar um vizinho
    double maxCosFound = minDotProductToFollow; 
    double tmpfx = 0, tmpfy = 0;

    for(uint i = 0; i < sample_count; i++) {
        double distance = scan[i];
        // Filtro para ignorar leituras vazias ou muito próximas
        if (distance < influence && distance > 0.3) { 
            double bearing = getBearing(i);
            double dx = cos(m_th + bearing);
            double dy = sin(m_th + bearing);

            // Produto escalar entre a direção do laser e o objetivo
            double dot = dx * GoalFx + dy * GoalFy;
            
            if (dot > maxCosFound) {
                maxCosFound = dot;
                tmpfx = Ka * dx;
                tmpfy = Ka * dy;
            }
        }
    }

    // 2. Decisão: Se achou vizinho, segue. Senão, vai pro alvo.
    if (tmpfx != 0 || tmpfy != 0) {
        fx = tmpfx;
        fy = tmpfy;
    } else {
        fx = GoalFx;
        fy = GoalFy;
    }
}