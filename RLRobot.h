#ifndef RLROBOT_H
#define RLROBOT_H

#include "Robot.h"
#include "TRVF.h"
#include "SQF.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>      
#include <arpa/inet.h> 
#include <unistd.h> 

class RLRobot : public Robot
{
public:
    static const int NUM_SECTORS = 12;
    static const int STATE_SIZE = 4 + NUM_SECTORS;
    RLRobot(ConfigFile *confFile);

    void init(int id);
    void mainLoop(); 
    int communicateWithPython(double state[STATE_SIZE]);

    void followNeighbor(); // Método para seguir um vizinho, se necessário.
    void applyAction(int action); // Método para aplicar a ação recebida do agente de RL.
    void collectState(double state[STATE_SIZE]); // Método para coletar o estado atual do robô e armazená-lo no array fornecido.
    
private:


    ConfigFile *cf; // Guardado para leitura de parametros (ex: rlPort) dentro de init().
    struct sockaddr_in serv_addr;
    int client_fd;
    int currentAction; // State of the probabilistic finite state machine.
    double minDotProductToFollow; // Limite para seguir um vizinho, baseado no produto escalar mínimo.
    bool socketConnected, targetSaved; // Flag para verificar se a conexão com o agente de RL está ativa.
    double initialTargetX, initialTargetY; 
 


    Pose start_pose; // Armazena a pose inicial do robô para referência.
    void reconnect(); // Método para tentar reconectar ao agente de RL caso a conexão seja perdida.

    int skipCounter; // Contador para controlar a frequência de decisões do agente de RL.
    int persistence; // Número de iterações para persistir na mesma ação antes de consultar o agente de RL novamente.

};

#endif