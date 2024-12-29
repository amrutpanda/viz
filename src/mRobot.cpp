#include "mRobot.h"


namespace mviz
{
    mRobot::mRobot(std::string robotname, std::string urdf_file, Ogre::SceneNode* root_node)
    {
        robot_name = robotname;
        _urdf_file = urdf_file;
        rootNode = root_node;
        _urdf = urdf::parseURDFFile(_urdf_file);

        // urdf::LinkConstSharedPtr linkPtr = _urdf->getRoot();
        // urdf_link = linkP

        urdf_root_link = _urdf->getRoot();
        _robot_name = _urdf->getName();
        
    //    urdf::LinkSharedPtr& l = const_cast<urdf::LinkSharedPtr&>(_urdf->getRoot());
    }

    // void mRobot::convertRobotLinkToOgreNode(urdf::LinkSharedPtr link, Ogre::SceneNode* sNode)
    // {
    //     // To be implemented.
    // }

    void mRobot::convertLinkConstSharedPtrTomRobotLink(urdf::LinkConstSharedPtr ulink, mRobotLink* rlink)
    {
        
        urdf::Mesh* m = dynamic_cast<urdf::Mesh*> (ulink->visual->geometry.get());
        if (m == NULL)
        {
            std::cout << "At this moment only Mesh objects are supported." << std::endl;
            throw std::runtime_error("cannot convert the link to Mesh type object. link name : " + ulink->name);

        }

        rlink->type = Type::MESH; // this is redundant.
        rlink->mesh_file_name = std::filesystem::canonical(m->filename).string();
        
        rlink->scale << m->scale.x , m->scale.y, m->scale.z;
        
    }


    void mRobot::convertLinkSharedPtrTomRobotLink(urdf::LinkSharedPtr ulink, mRobotLink* rlink)
    {
        urdf::Mesh* m = dynamic_cast<urdf::Mesh*> (ulink->visual->geometry.get());
        if (m == NULL)
        {
            std::cout << "At this moment only Mesh objects are supported." << std::endl;
            throw std::runtime_error("cannot convert the link to Mesh type object. link name : " + ulink->name);

        }
        rlink->type = Type::MESH; // this is redundant.
        rlink->mesh_file_name = std::filesystem::canonical(m->filename).string();
        
        rlink->scale << m->scale.x , m->scale.y, m->scale.z;
        
    }

    void mRobot::createOgreNodeFromLinkSharedPtr(urdf::LinkSharedPtr ulink, Ogre::SceneNode* ogNode)
    {
        if (ulink->child_links.size() == 0)
        {
            Ogre::SceneNode* childNode = ogNode->createChildSceneNode();
            link_names.push_back(ulink->name);
            ogreNodes.push_back(childNode);
            // create a mRobotLink object.
            mRobotLink* rlink = new mRobotLink();
            object_ptrs.push_back(rlink);
            // convert the urdf link object to mRobotLink object.
            convertLinkSharedPtrTomRobotLink(ulink,rlink);
            rlink->setSceneNode(childNode);
            creatMeshFromFile(rlink->mesh_file_name, rlink->entity_name);
        }
        else 
        {
            for (size_t i = 0; i < ulink->child_links.size(); i++)
            {
                Ogre::SceneNode* childNode = ogNode->createChildSceneNode();
                link_names.push_back(ulink->name);
                ogreNodes.push_back(childNode);
                // create a mRobotLink object.
                mRobotLink* rlink = new mRobotLink();
                object_ptrs.push_back(rlink);
                // convert the urdf link object to mRobotLink object.
                convertLinkSharedPtrTomRobotLink(ulink,rlink);
                creatMeshFromFile(rlink->mesh_file_name,rlink->entity_name);
                // recursive call to this function.
                createOgreNodeFromLinkSharedPtr(ulink->child_links[i],childNode);
            }
            
        }
        
    }

} // namespace mviz
