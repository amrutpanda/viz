
void mRobot::convertCollsionLinkPtrTomRobotLink(urdf::Link* ulink, mRobotLink* rlink)
{
    std::cout << "Collision array size: " << ulink->Collision_array.size() << std::endl;
    for (int i = 0; i < ulink->Collision_array.size(); ++i)
    {
        urdf::CollisionSharedPtr cptr = ulink->Collision_array[i];
        // if not Collision component exists.
        if (cptr.get() != nullptr)
        {
            // std::cout <<   << std::endl;
            bool rqd_mat_loading = true;
            std::string mesh_name;
            double x,y,z,qw,qx,qy,qz;
            x = cptr->origin.position.x;
            y = cptr->origin.position.y;
            z = cptr->origin.position.z;
            cptr->origin.rotation.getQuaternion(qx,qy,qz,qw);
            unsigned int type = getUrdfGeometryType(cptr->geometry);

            switch (type)
            {
            case urdf::Geometry::BOX :
                {
                    double l,b,h;
                    mesh_name = "mCube.mesh";
                    
                    urdf::Box* box_ptr = dynamic_cast<urdf::Box*>(cptr->geometry.get());
                    l = box_ptr->dim.x;
                    b = box_ptr->dim.y;
                    h = box_ptr->dim.z;

                    rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                            Ogre::Vector3(l,b,h));
                    break;
                } 
            case urdf::Geometry::CYLINDER :
                {
                    double r,h;
                    mesh_name = "mCylinder.mesh";
                    
                    urdf::Cylinder* c_ptr = dynamic_cast<urdf::Cylinder*>(cptr->geometry.get());

                    r = c_ptr->radius;
                    h = c_ptr->length;
                    rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                            Ogre::Vector3(r,r,h/10));
                    
                    break;
                }
            case urdf::Geometry::SPHERE :
                {
                    double r;
                    mesh_name = "mSphere.mesh";
                    urdf::Sphere* s_ptr = dynamic_cast<urdf::Sphere*>(cptr->geometry.get());

                    r = s_ptr->radius;

                    rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                            Ogre::Vector3(r,r,r));

                    break;
                }
            
            case urdf::Geometry::MESH :
                {
                    std::string mesh_file_name;
                    urdf::Mesh* m_ptr = dynamic_cast<urdf::Mesh*>(cptr->geometry.get());
                    
                    std::filesystem::path dir_path(_urdf_file), filepath(m_ptr->filename);
                    mesh_file_name = dir_path.parent_path().append(filepath.string());
                    createMeshFromFile(mesh_file_name,mesh_name);

                    // check whether the file extension is stl and set material loading flag true;
                    if (filepath.extension() == ".stl" )
                        rqd_mat_loading = true;
                    else
                        rqd_mat_loading = false;


                    rlink->attachChildMesh(scnMgr,mesh_name,Ogre::Vector3(x,y,z),Ogre::Quaternion(qw,qx,qy,qz),
                                            Ogre::Vector3(m_ptr->scale.x, m_ptr->scale.y, m_ptr->scale.z));

                    if (filepath.extension() == ".dae" || filepath.extension() == ".DAE")
                    {
                        // A patch to handle orientatin mismatch issue with Collada files.
                        // Change the angle and axes if you see mesh pose discrepancy.
                        std::cout << "Found a Collada file." << std::endl;
                        // rlink->getChildMeshNode(mesh_name)->rotate(Ogre::Quaternion(Ogre::Degree(90),Ogre::Vector3::UNIT_X));
                        rlink->getChildMeshNode(mesh_name)->rotate(Ogre::Quaternion(Ogre::Degree(mfAngle),mfAxis));
                    }
                    break;
                }
            
            default:
                {
                    rqd_mat_loading = false;
                    std::cout << "No Collision Info found\n";
                    break;
                }
            }
       
            // set material 
            if (rqd_mat_loading && (cptr->material.get() != nullptr))
            {
                std::cout << "Preparing to load material..." << std::endl;
                Ogre::MaterialPtr pmat;
                Ogre::ResourceManager::ResourceCreateOrRetrieveResult res;
                if (cptr->material->name.empty())
                    cptr->material->name = "Default";
                pmat = Ogre::MaterialManager::getSingleton().getByName(cptr->material->name);
                if (pmat.get() == nullptr)
                    pmat = Ogre::MaterialManager::getSingleton().create(cptr->material->name,"UserData");
                // TO-DO: Need to fix a bug here.
                Ogre::ColourValue _color = Ogre::ColourValue(cptr->material->color.r,
                                                            cptr->material->color.g,
                                                            cptr->material->color.b);
                pmat->getTechnique(0)->getPass(0)->setAmbient(_color);
                pmat->getTechnique(0)->getPass(0)->setDiffuse(_color);
                
    
                if (!cptr->material->texture_filename.empty())
                {
                    pmat->getTechnique(0)->getPass(0)->createTextureUnitState(cptr->material->texture_filename);
                }
                // retrieve the attached entity from the child mesh node;
                Ogre::Entity* ent = static_cast<Ogre::Entity*>(rlink->getChildMeshNode(mesh_name)->getAttachedObject(0));
                ent->setMaterial(pmat);
            }
            else if (rqd_mat_loading)
            {
                // std::cout << "Loading default material." << std::endl;
                // Ogre::Entity* ent = static_cast<Ogre::Entity*>(rlink->getChildMeshNode(mesh_name)->getAttachedObject(0));
                // ent->setMaterialName("BaseWhiteNoLighting");
            }
            else
                std::cout << "No material required \n";
            // store the mesh names.

            rlink->meshes.push_back(mesh_name);
        }
    }   
    
}