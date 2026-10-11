/* Adapted from http://www.adp-gmbh.ch/cpp/config_file.html */

#ifndef __CONFIGFILE_H__
#define __CONFIGFILE_H__

#include <string>
#include <map>

using namespace std;

class ConfigFile {
private:
  // A map from variable name to content.
  map<string,string> content_;
  
  // File name being used.
  string configFileName;
  
public:
  /* Opens a config file and creates a map from variable to content.
   * Argument:
   *   configFile: a string with the file name.*/
  ConfigFile(string const& configFile);

  /* Reads the value of a variable. If it does not exists, this method throws a const char* exception.
   * Argument:
   *   entry: variable name to get the value.
   * Return: the string value of the given variable.
   * Throws: a const char*
   */
  string const& valueOf(string const& entry) const;
};

#endif
