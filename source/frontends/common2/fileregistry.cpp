#include "StdAfx.h"

#include "frontends/common2/fileregistry.h"
#include "frontends/common2/ptreeregistry.h"
#include "frontends/common2/programoptions.h"

#include "Log.h"
#include "linux/windows/files.h"
#include "frontends/qt/applicationname.h"

#include <boost/property_tree/ini_parser.hpp>

#include <sys/stat.h>

namespace
{

  void parseOption(const std::string & s, std::string & path, std::string & value)
  {
    const size_t pos = s.find('=');
    if (pos == std::string::npos)
    {
      throw std::runtime_error("Invalid option format: " + s + ", expected: section.key=value");
    }
    path = s.substr(0, pos);
    std::replace(path.begin(), path.end(), '_', ' ');
    value = s.substr(pos + 1);
    std::replace(value.begin(), value.end(), '_', ' ');
  }

  std::string getHomeDir()
  {
    const char* homeDir = getenv("HOME");
    if (!homeDir)
    {
      throw std::runtime_error("${HOME} not set, cannot locate configuration file");
    }

    return std::string(homeDir);
  }

  class Configuration : public common2::PTreeRegistry
  {
  public:
    Configuration(const std::string & filename, const bool saveOnExit);
    ~Configuration();

    void addExtraOptions(const std::vector<std::string> & options);

  private:
    const std::string myFilename;
    const bool mySaveOnExit;
  };

  Configuration::Configuration(const std::string & filename, const bool saveOnExit) : myFilename(filename), mySaveOnExit(saveOnExit)
  {
    if (GetFileAttributes(myFilename.c_str()) != INVALID_FILE_ATTRIBUTES)
    {
      boost::property_tree::ini_parser::read_ini(myFilename, myINI);
    }
    else
    {
      LogFileOutput("Registry: configuration file '%s' not found\n", filename.c_str());
    }
  }

  Configuration::~Configuration()
  {
    if (mySaveOnExit)
    {
      boost::property_tree::ini_parser::write_ini(myFilename, myINI);
    }
  }

  void Configuration::addExtraOptions(const std::vector<std::string> & options)
  {
    for (const std::string & option : options)
    {
      std::string path, value;
      parseOption(option, path, value);
      myINI.put(path, value);
    }
  }

}

namespace common2
{

  std::string GetConfigFile(const std::string & filename)
  {
    const std::string dir = getHomeDir() + "/.applewin";
    const int status = mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (!status || (errno == EEXIST))
    {
      return dir + "/" + filename;
    }
    else
    {
      const char * s = strerror(errno);
      LogFileOutput("No registry. Cannot create %s in %s: %s\n", filename.c_str(), dir.c_str(), s);
      return std::string();
    }
  }

  void InitializeFileRegistry(const EmulatorOptions & options)
  {
    const std::string homeDir = getHomeDir();

    std::string filename;
    bool saveOnExit;

    if (options.useQtIni)
    {
      filename = homeDir + "/.config/" + ORGANIZATION_NAME + "/" + APPLICATION_NAME + ".conf";
      saveOnExit = false;
    }
    else
    {
      filename = GetConfigFile("applewin.conf");
      saveOnExit = !filename.empty() && options.saveConfigurationOnExit;
    }

    std::shared_ptr<Configuration> config(new Configuration(filename, saveOnExit));
    config->addExtraOptions(options.registryOptions);

    Registry::instance = config;
  }

}
