#!/usr/bin/env python3

import numpy as np
import auto_targetter

auto_targetter = auto_targetter.BraccioObjectTargetInterface()

auto_targetter.load_calibrate()

def main():
    with open ("choosen.txt") as g:
        groups = np.array([[x for x in line.split()] for line in g])
        print(groups)

    #creating an array of the sherds on the table with (class, conf_lev, (x,y)_cent, link_name)
    with open ("posizioni_mm.txt") as pos:
        posizioni = np.array([[x for x in line.split()] for line in pos])

    link_choose = []

    #creating a list with the choosen link name
    for i in range(len(posizioni)):
        if (np.in1d(posizioni[i,0], groups)): 
            lk = (posizioni[i, 0].astype(int), posizioni[i,4], np.where([groups==posizioni[i,0]])[1])
            print(lk)
            # print(np.where([groups==posizioni[i,0]])[0].astype(int))
            link_choose.append(lk)
                
    # for j in range(len(link_choose)):
    #     print(f"link choose is: {link_choose[j,3]}")  
    link_array = np.array(link_choose)
    print(f"selected is {np.array2string(link_array)}")

    for j in range(len(link_array)):
        inp_ch = link_array[j,1].astype(str) + "::link"
        print(inp_ch)
        auto_targetter.get_link_choose(inp_ch)
        bowl_ch = np.array2string(link_array[j,2])
        bowl_ch = bowl_ch.replace('[','').replace(']','')
        bowl_ch = "go_to_home_" + bowl_ch
        auto_targetter.go_to_target('top', bowl_ch)
        print(bowl_ch)
        

if __name__ == "__main__":
  main()