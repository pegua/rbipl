/**
 * Copyright  (C) 2013  CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */ 
     
#ifndef RBIPL_TREE_ROTATION_HPP
#define RBIPL_TREE_ROTATION_HPP

#include <Eigen/Core>

#include <kdl/tree.hpp>


namespace RBIPL {    
    
bool addTreeRecursive(SegmentMap::const_iterator old_tree_root, Tree & new_tree, const std::string& hook_name)
{
    //get iterator for root-segment
    SegmentMap::const_iterator child;
    //try to add all of root's children
    for (unsigned int i = 0; i < root->second.children.size(); i++) {
        child = root->second.children[i];
        //Try to add the child
        if (new_tree.addSegment(child->second.segment, hook_name)) {
            //if child is added, add all the child's children
            if (!(new_tree.addTreeRecursive(child, new_tree, child->first)))
                //if it didn't work, return false
                return false;
        } else
            //If the child could not be added, return false
            return false;
    }
    return true;
}
    
    
    /**
     * Rotate a KDL::Tree to obtain an equivalent rapresentation, but with a different root link.
     *
     * To perform the rotation it is necessary that the first segment of the tree has a Joint::None joint.
     * 
     * @param old_tree the input KDL::Tree to rotate 
     * @param new_tree the output rotated KDL::Tree
     * @param new_root_name the name of the link to use as a new root
     * 
     * @return true if all went well, false otherwise
     * 
     * \todo add error checking
     */
    bool tree_rotation(const KDL::Tree & old_tree, KDL::Tree & new_tree, const std::string& new_root_name) {
    {
        SegmentMap::iterator dummy_root;
        SegmentMap::iterator old_root; //old *real* root 
        SegmentMap::iterator new_root; //new *real* root
        SegmentMap segments;
        unsigned int i; 

        
        old_tree.getSegments(segments);
        new_root = segments.find(new_root_name);
        //check if new root exists
        if (new_root == segments.end())
            return false;
        
        //check that the tree has a root segment with only one child, connected 
        //with a Joint::None (a fixed joint) (standard in humanoids URDF)
        old_tree.getRootSegment(dummy_root);
        if( dummy_root->second.children.size() != 1 ) 
            return false;
            
        old_root = dummy_root->second.children[0];
        if( old_root->second.segment.getJoint().getType() != Joint::None )
            return false;
        
        //The dummy_root is moved 
        new_tree = KDL::Tree(dummy_root->first);
        
        //the new root is the specified segment, but with Joint::None as the joint
        new_tree.addSegment(Segment(new_root_name,
                                    Joint(Joint::None),
                                    Frame::Identity(),
                                    new_root->second.segment.getInertia()),
                            new_root_name) 
                            
        //Now that the new root is inserted, the successor subchain can be simply added
        //Not efficient, but to avoid code duplication
        //addTreeRecursive method is private, a copy is done
        addTreeRecursive(new_root,new_tree,new_root->first);
        
        //Now the other part of the tree has to be added 
        
        
            

    }

}
             
#endif
