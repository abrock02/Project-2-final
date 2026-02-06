#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 12:37:32 2021

@author: kendrick shepherd
"""

import sys

import Geometry_Operations as geom

# Determine the unknown bars next to this node
def UnknownBars(node):
    unknown_bars = []
    for bar in node.bars:
        if bar.is_computed == False:
            unknown_bars.append(bar)
    return unknown_bars

# Determine if a node if "viable" or not
def NodeIsViable(node):
    unknowns_list = UnknownBars(node)
    num_unknowns = len(unknowns_list)

    if num_unknowns == 0:
        return False
    elif num_unknowns > 2:
        return False
    else:
        return True
    
# Compute unknown force in bar due to sum of the
# forces in the x direction
def SumOfForcesInLocalX(node, local_x_bar):
    vec_local_x = geom.BarNodeToVector(node, local_x_bar)
    
    force_sum = 0
    
    net_fx = node.GetNetXForce()
    net_fy = node.GetNetYForce()
    
    force_sum += net_fx * geom.CosineVectors(vec_local_x, [1, 0])
    force_sum += net_fy * geom.CosineVectors(vec_local_x, [0, 1])
    
    for bar in node.bars:
        if bar.is_computed:
            vec_this_bar = geom.BarNodeToVector(node, bar)
            force_sum += bar.axial_load * geom.CosineVectors(vec_local_x, vec_this_bar)
            
    local_x_bar.axial_load = -force_sum
    local_x_bar.is_computed = True
    return

# Compute unknown force in bar due to sum of the 
# forces in the y direction
def SumOfForcesInLocalY(node, unknown_bars):
    local_x_bar = unknown_bars[0]
    
    vec_local_x = geom.BarNodeToVector(node, local_x_bar)
    
    force_sum = 0
    net_fx = node.GetNetXForce()
    net_fy = node.GetNetYForce()
    
    force_sum += net_fx * geom.SineVectors(vec_local_x, [1, 0])
    force_sum += net_fy * geom.SineVectors(vec_local_x, [0, 1])
    
    for bar in node.bars:
        if bar.is_computed:
            vec_this_bar = geom.BarNodeToVector(node, bar)
            sin_theta = geom.SineVectors(vec_local_x, vec_this_bar)
            force_sum += bar.axial_load * sin_theta
            
    other_bar = unknown_bars[-1] 
    vec_other_bar = geom.BarNodeToVector(node, other_bar)
    sin_other = geom.SineVectors(vec_local_x, vec_other_bar)
    
    if abs(sin_other) > 1e-9:
        other_bar.axial_load = -force_sum / sin_other
        other_bar.is_computed = True
    return
    
def DoIHaveAnUnknownMember(nodes):
    for node in nodes:
        unknown_bars = UnknownBars(node)
        if len(unknown_bars) > 0:
            return True
        
    return False
def IterateUsingMethodOfJoints(nodes,bars):
    counter = 0
    while DoIHaveAnUnknownMember(nodes) == True:
        for node in nodes:
            if NodeIsViable(node):
                current_unknowns = UnknownBars(node)
                SumOfForcesInLocalY(node, current_unknowns)
                remaining_unknowns = UnknownBars(node)
                
                if len(remaining_unknowns) > 0:
                    SumOfForcesInLocalX(node, remaining_unknowns[0])
        counter += 1
        if counter > len(nodes) + 1:
            sys.exit("Too many iterations")
    return bars

    
    