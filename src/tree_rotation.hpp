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
     */
    bool tree_rotation(const KDL::Tree & old_tree, KDL::Tree & new_tree, const std::string& new_root_name) {
    {
        SegmentMap::iterator old_root;
        SegmentMap::iterator new_root;
        SegmentMap segments;
        
        old_tree.getSegments(segments);
        new_root = segments.find(new_root_name);
        //check if new root exists
        if (new_root == segments.end())
            return false;
        
        //check that the tree has a root segment with a Joint::None
        old_tree.getRootSegment(old_root);
        if( old_root.getJoint().getType() != Joint::None )
            return false;
            
        new_tree = KDL::Tree();
        
        
        //the new root is the same segment, but with Joint::None as the joint
        new_tree.addSegment(Segment(Joint(Joint::None),
                                    Frame::Identity(),
                                    new_root.),
                            new_root_name)
        
        
            

    }

}
             
#endif
