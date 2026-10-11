#include "ConfigFile.h"

#include <fstream>

using namespace std;


/* Removes trailing and starting whitespaces or other delimiters.
 * Argument:
 *   source: a string to be trimmed.
 *   delims (optional): a string containg the delimiters.
 * Return: the trimmed string.
 */
string trim(string const& source, char const* delims = " \t\r\n") {
  string result(source);
  string::size_type index = result.find_last_not_of(delims);
  if(index != string::npos)
    result.erase(++index);

  index = result.find_first_not_of(delims);
  if(index != string::npos)
    result.erase(0, index);
  else
    result.erase();
  return result;
}


/* Opens a config file and creates a map from variable to content.
 * Argument:
 *   configFile: a string with the file name.*/
ConfigFile::ConfigFile(string const& configFile) {
  configFileName = configFile;
  ifstream file(configFile.c_str());
  string line;
  string name;
  string value;
  string inSection;
  int posEqual;
  while (getline(file,line)) {
    if (! line.length()) continue;

    if (line[0] == '#') continue;
    if (line[0] == ';') continue;

    posEqual=line.find('=');
    name  = trim(line.substr(0,posEqual));
    value = trim(line.substr(posEqual+1));

    content_[name]=value;
  }
  file.close();
}

/* Reads the value of a variable. If it does not exists, this method throws a const char* exception.
 * Argument:
 *   entry: variable name to get the value.
 * Return: the string value of the given variable.
 * Throws: a const char*
 */
string const& ConfigFile::valueOf(string const& entry) const {
  map<string,string>::const_iterator ci = content_.find(entry);
  if (ci == content_.end()){ 
    string str = "String " + entry + " does not exist in the config file \'" + configFileName + "\'.";
    throw str;
  }
  return ci->second;
}
