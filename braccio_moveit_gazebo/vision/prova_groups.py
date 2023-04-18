import numpy as np

# choosen_file = os.path.join(vision_path, "choosen.txt")
    # with open (os.path.join(vision_path, "choosen.txt")) as g:
with open ("choosen.txt") as g:
    groups = np.array([[x for x in line.split()] for line in g])
    print(groups)

# choosen_array 

#creating an array of the sherds on the table with (class, conf_lev, (x,y)_cent, link_name)
with open ("posizioni_mm.txt") as pos:
    posizioni = np.array([[x for x in line.split()] for line in pos])
    print(posizioni)

link_choose = []

#creating a list with the choosen link name
for i in range(len(posizioni)):
    if (np.in1d(posizioni[i,0], groups)): 
        lk = (posizioni[i, 0].astype(int), posizioni[i,4], np.array2string(np.where([groups==posizioni[i,0]])[1]))
        print(lk)
        # print(np.where([groups==posizioni[i,0]])[0].astype(int))
        link_choose.append(lk)
            
# for j in range(len(link_choose)):
#     print(f"link choose is: {link_choose[j,3]}")  
print(link_choose)
link_array = np.array(link_choose)
print(f"selected is {np.array2string(link_array)}")