#include "cli/common.h"

using namespace lime;

SDRDevice               *device = nullptr;
SDRDevice::SDRConfig    config;

static int printHelp(void)
{
    std::cout << "Usage LimeFLASH [options] program.bin" << std::endl;
    std::cout << "    --help\t\t\t This help" << std::endl;
    std::cout << "    --index <i>\t\t\t Specifies device index (0-...). Required if multiple devices" << std::endl;
    std::cout << "    --target <TARGET>\t\t <TARGET> programming. \"-\" to list targets" << std::endl;

    return EXIT_SUCCESS;
}

void listTargets (void)
{
    std::cout <<"Possible device targets :" << std::endl;
    auto d = device->GetDescriptor();
    
}

bool CallBack (size_t bsent, size_t btotal, const char* statusMessage)
{
    return true;
}


/***********************************************************************
 * main entry point
 **********************************************************************/
int main(int argc, char *argv[])
{
    static struct option long_options[] = {
        {"help", no_argument, 0, 'h'},
        {"index", required_argument, 0, 'i'},
        {"target", required_argument, 0, 't'},
        {0, 0, 0,  0}
    };

    int dev_index = -1;
    int long_index = 0;
    int option = 0;
    int target = -1;
    char *fn = NULL;
    std::vector<char> data;
    std::ifstream inf;
    
    while ((option = getopt_long_only (argc, argv, "", long_options, &long_index)) != -1)
    {
        switch (option)
        {
        case 'h': 
            return printHelp ();
        case 'i':
            if (optarg != NULL) dev_index = std::stod (optarg);
            break;
        case 't':
            if (optarg != NULL && optarg[0] != '-') 
               target = std::stod (optarg);
            break;
        }
    }
    if  (target < 0) {printHelp (); return EXIT_SUCCESS;}
    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0) 
    {
        std::cout << "No devices found" << std::endl;
        return -1;
    } else {
        if  (handles.size () > 1 && dev_index == -1)
        {
            std::cout << "Multiple devices detected and no --index option specified" << std::endl;
            return -1;
        } else
            dev_index = 0;
    }
    device = DeviceRegistry::makeDevice(handles.at(dev_index));
    
    
    
    
    if  (optind == argc) {printHelp (); return -1;}
//    if  (optind < argc) printf ("%s\n",argv[optind]);
printf ("%u %u\n",optind,argc);
    exit;
//    inf.open (fflash, std::ifstream::in | std::ifstream::binary);

    if  (!inf)
    {
        std::cout << "Unable to open file." << std::endl;
        return -1;
    }
    inf.seekg (0,std::ios_base::end);
    auto cnt = inf.tellg ();
    inf.seekg (0,std::ios_base::beg);
    std::cout << "File size : " << cnt << " bytes." << std::endl;
    data.resize (cnt);
    inf.read (data.data (),cnt);
    inf.close ();
  /*  if (device->UploadMemory (fram?0:1, data.data(), data.size(), CallBack))
    {
        std::cout << "Error programming device." << std::endl;
        return -1;
    }
    std::cout << "Ok." << std::endl;
*/
    return EXIT_SUCCESS;
}

