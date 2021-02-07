import os
import time
import numpy as np
import subprocess 


import scipy
def str2num(LineString,comment='#'):
  
  from io import StringIO as StringIO
  import re,numpy
  
  NumArray=numpy.empty([0],numpy.int16)
  NumStr=LineString.strip()
  #~ ignore comment string
  for cmt in comment:
 
    CmtRe=cmt+'.*$'
    NumStr=re.sub(CmtRe, " ", NumStr.strip(), count=0, flags=re.IGNORECASE)
  
  #~ delete all non-number characters,replaced by blankspace.
  NumStr=re.sub('[^0-9.e+-]', " ", NumStr, count=0, flags=re.IGNORECASE)
  
  #~ Remove incorrect combining-characters for double type.
  NumStr=re.sub('[.e+-](?=\s)', " ", NumStr.strip(), count=0, flags=re.IGNORECASE)
  NumStr=re.sub('[.e+-](?=\s)', " ", NumStr.strip(), count=0, flags=re.IGNORECASE)
  NumStr=re.sub('[e+-]$', " ", NumStr.strip(), count=0, flags=re.IGNORECASE)
  NumStr=re.sub('[e+-]$', " ", NumStr.strip(), count=0, flags=re.IGNORECASE)
  
  if len(NumStr.strip())>0:
    StrIOds=StringIO(NumStr.strip())
    NumArray= numpy.genfromtxt(StrIOds)
  
  return NumArray




##==================================MPC loop###############33

start_time = time.time()
cpptest="/home/jiatao/Dropbox/nmpc_pybullet/build/src/MPC_WALK.exe" #in linux without suffix .exe


FileLength = 4000

SensorValue = np.zeros([FileLength,12])

for i in range(0,FileLength):
    #### convert the double type to char type
    #a = 0.5
    #b = '1 2 \t' + str(a)   
    j=i
    b = str(j)

    if os.path.exists(cpptest):
        rc, out = subprocess.getstatusoutput(cpptest+' '+b)
        # print(out)
        donser=str2num(out)
        if len(donser) ==12:
            SensorValue[i] = donser
            print(donser)


        # f=os.popen(cpptest+' '+b)
        # data=f.readlines() #read the C++ printf or cout content
        # f.close()
        # print(data)


    #    print(donser)  
    #    dimen = donser.shape
    #    print(dimen)

    


#### read file for fetch data: return a list,     
#    f=os.popen(cpptest+' '+b)
#    data=f.readlines() #read the C++ printf or cout content
#    f.close()
	
#os.system(cpptest+' '+ '2')


np.savetxt("MPC_WALK_gait_reference.txt", SensorValue, fmt="%f")



end_time = time.time()
print('totally cost',end_time-start_time)


