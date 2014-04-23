/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Jeremie Papon
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author : Jason Owens
 *  Email  : pcl.jlowenz@xoxy.net
 */

#include <boost/foreach.hpp>

namespace pcl {
  namespace octree {

    template<typename LeafContainerT,
	     typename BranchContainerT>
    void 
    OctreeLeafSignal<LeafContainerT,BranchContainerT>::connect(const typename OctreeObserverT::Ptr& observer)
    {
      observers_.push_back(observer);
    }

    template<typename LeafContainerT,
	     typename BranchContainerT>
    void 
    OctreeLeafSignal<LeafContainerT,BranchContainerT>::disconnect(const typename OctreeObserverT::Ptr& observer)
    {
      typename ObserverList::iterator i;
      i = std::find(observers_.begin(), observers_.end(), observer);
      if (i != observers_.end()) {
	observers_.erase(i);
      }
    }
    
    // signals
    template<typename LeafContainerT,
	     typename BranchContainerT>
    void 
    OctreeLeafSignal<LeafContainerT,BranchContainerT>::leafCreated(LeafContainerT* leaf)
    {
      BOOST_FOREACH(typename OctreeObserverT::Ptr o, observers_) {
	o->leafCreated(leaf);
      }
    }
    
    // hmmm - this one may not work
    template<typename LeafContainerT,
	     typename BranchContainerT>
    void 
    OctreeLeafSignal<LeafContainerT,BranchContainerT>::leafDestroyed(LeafContainerT* leaf)
    {
      BOOST_FOREACH(typename OctreeObserverT::Ptr o, observers_) {
	o->leafDestroyed(leaf);
      }
    }

  }
}
