
#include "mGraphics.h"

// #include <Ogre.h>
// #include <urdf_parser/urdf_parser.h>

namespace mviz
{
    void mGraphics::readFile(std::string FilePath)
    {
        urdf = urdf::parseURDFFile(FilePath);
        
        urdf::LinkConstSharedPtr root = urdf->getRoot();
        
        
    }

    urdf::ModelInterfaceSharedPtr mGraphics::getUrdfObject()
    {
        return urdf;
    }


    void mGraphics::urdf_to_ogre_converter(Ogre::SceneManager* scm)
    {
        // to be implemented.
        urdf::LinkSharedPtr link;

    }

    const std::string& mGraphics::getName()
    {
        return urdf->getName();
    }

    // ....................... functions not part of Graphics object........................ //
    void creatMeshFromFile(std::string filepath, Ogre::String& MeshName)
    {
        if (!Ogre::ResourceGroupManager::getSingleton().resourceGroupExists("User"))
        {
            Ogre::ResourceGroupManager::getSingleton().createResourceGroup("User");
        }
        std::filesystem::path path = std::filesystem::canonical(filepath);
        std::cout << path.parent_path() << std::endl;
        std::filesystem::recursive_directory_iterator it(path.parent_path());
        std::filesystem::recursive_directory_iterator end;

        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(path.parent_path().generic_string(),
                                                                        "FileSystem","User");
        while (it != end)
        {
            if (it->is_directory())
            {
                Ogre::ResourceGroupManager::getSingleton().addResourceLocation(it->path().string(),"FileSystem","User");
            }   
            ++it;
        }

        // std::cout << "Intilising resource group User" << std::endl;
        Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("User");
        // std::cout << "loading resource group User " << std::endl;
        Ogre::ResourceGroupManager::getSingleton().loadResourceGroup("User");

        Ogre::String basename, ext, pathname;
        Ogre::StringUtil::splitFullFilename(filepath,basename,ext,pathname);

        std::cout << basename + "." + ext << std::endl;

        // std::cout << "Creating Codec object." << std::endl;
        auto codec = Ogre::Codec::getCodec(ext);

        AssOptions opts;
        opts.source = basename + "." + ext ;

        Ogre::MeshPtr m = Ogre::MeshManager::getSingleton().createManual(basename + "." + ext, "User");
        m->getUserObjectBindings().setUserAny("_AssimpLoaderOptions", opts.options);
        // std::cout << "Decoding the obj file to mesh" << std::endl;
        codec->decode(Ogre::Root::openFileStream(opts.source), m.get());

        // std::cout << "decoding complete." << std::endl;

        MeshName = basename + "." + ext;

    }


    void say_hello()
    {
        std::cout << "Hello. " << std::endl;
    }
} // namespace mviz




