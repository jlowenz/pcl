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

#ifndef PCL_OCTREE_OBSERVER_H_
#define PCL_OCTREE_OBSERVER_H_

#include <pcl/console/print.h>
#include <pcl/common/geometry.h>
#include <pcl/octree/boost.h>
#include <pcl/octree/octree_key.h>

#include <list>


namespace pcl
{

  namespace octree
  {
    
    template<typename LeafContainerT = int,
	     typename BranchContainerT = OctreeContainerEmpty >
    class OctreeObserver
    {
    public:
      typedef boost::shared_ptr<OctreeObserver<LeafContainerT,BranchContainerT> > Ptr;
      typedef LeafContainerT LeafContainer;
      //typedef BranchContainerT BranchContainer;

      virtual void leafCreated(LeafContainerT* leaf) = 0;
      virtual void leafDestroyed(LeafContainerT* leaf) = 0;
    };
    

    template<typename LeafContainerT = int,
	     typename BranchContainerT = OctreeContainerEmpty >
    class OctreeLeafSignal 
    {
    public:
      typedef OctreeObserver<LeafContainerT,BranchContainerT> OctreeObserverT; 
      
      void connect(const typename OctreeObserverT::Ptr& observer);
      void disconnect(const typename OctreeObserverT::Ptr& observer);

      // signals
      void leafCreated(LeafContainerT* leaf);

      // hmmm - this one may not work
      void leafDestroyed(LeafContainerT* leaf);

    private:
      // perhaps this could be a set?
      typedef std::list<typename OctreeObserverT::Ptr> ObserverList;
      ObserverList observers_;
    };   

  }
}

#include <pcl/octree/impl/octree_observer.hpp>

#endif
