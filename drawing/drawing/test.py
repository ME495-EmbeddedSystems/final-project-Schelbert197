import numpy as np

Tla = np.array([[0, 1, 0],
                [1, 0, 0],
                [0, 0, -1]])
                
                
s = np.sin(-np.pi/3)   
c=   np.cos(-np.pi/3)           
a =  np.array([[c, 0, s],
                [0, 1, 0],
                [-s, 0, c]])
                
print(Tla@a)
                 