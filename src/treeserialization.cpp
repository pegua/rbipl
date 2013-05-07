/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
  
#include "treeserialization.hpp"
#include <kdl/joint.hpp>
#include <algorithm>
#include <cassert>
#include <iostream>

namespace KDL {
    
    void TreeSerialization::addDFSrecursive(SegmentMap::const_iterator current_el,int & link_cnt)
    {
        if( current_el->second.segment.getJoint().getType() != Joint::None ) {
            joints[current_el->second.q_nr] = current_el->first;
        }
        links[link_cnt] = current_el->first;
        link_cnt++;
                
        for( unsigned int i=0; i < current_el->second.children.size(); i++ ) {
            addDFSrecursive(current_el->second.children[i],link_cnt);
        }
        
    }
    
    TreeSerialization::TreeSerialization() 
    {
        links.clear();
        joints.clear();
    }

    TreeSerialization::~TreeSerialization() {}
    
    TreeSerialization::TreeSerialization(const Tree & tree) 
    {
        links.resize(tree.getNrOfSegments());
        joints.resize(tree.getNrOfJoints());
        
        
        SegmentMap::const_iterator root;
        SegmentMap::const_iterator child;
        
        int link_cnt = 0;
        
        tree.getRootSegment(root);
        for( unsigned int i=0; i < root->second.children.size(); i++ ) {
            addDFSrecursive(root->second.children[i],link_cnt);
        }
        
        assert(this->is_consistent(tree));
    }
        
    TreeSerialization::TreeSerialization(std::vector<std::string> & links_in, std::vector<std::string> & joints_in) 
    {
        links = links_in;
        joints = joints_in;
    }
    
    int TreeSerialization::getJointId(std::string joint_name)
    {
        std::vector<std::string>::iterator it;
        it = std::find(joints.begin(),joints.end(),joint_name);
        if( it != joints.end() ) {
            return it - joints.begin();
        } else {
            return -1;
        }
    }
        
    int TreeSerialization::getLinkId(std::string link_name)
    {
        std::vector<std::string>::iterator it;
        it = std::find(links.begin(),links.end(),link_name);
        if( it != links.end() ) {
            return it - links.begin();
        } else {
            return -1;
        }
    }
    
    std::string TreeSerialization::getJointName(int joint_id)
    {
        return joints[joint_id];
    }
    
    std::string TreeSerialization::getLinkName(int link_id)
    {
        return links[link_id];
    }
    
    bool TreeSerialization::is_consistent(const Tree & tree)
    {
        SegmentMap::const_iterator seg;
        if( tree.getNrOfJoints() != joints.size() || tree.getNrOfSegments() !=  links.size() ) return false;
        
        
        unsigned int i;
        
        for(i = 0; i < links.size(); i++ ) {
            if( !tree.getSegment(links[i],seg) ) return false;
        }
        
        
        for(i = 0; i < joints.size(); i++ ) {
            if( !tree.getSegment(joints[i],seg) ) return false;
            if( seg->second.segment.getJoint().getType() == Joint::None ) return false;

        }
        return true;
        
    }
            
    int TreeSerialization::getNrOfSegments() 
    {
        return links.size();
    }
        
    int TreeSerialization::getNrOfJoints()
    {
        return joints.size();
    }

}
