# iarc_detection_modules
Detection Packages for IARC Mission 9  
Results of using SURF/SIFT as feature detectors   
Conclusions:-  
1)SURF is better since it takes less time an more good matches  
2)The best distance at which feature detection should be performed is at(x,y,z) =(5,1,1.8)(The text is actual at (5,0.7,1.8))  
SURF   
|Index     |No of Good matches    |Descriptor size   |time_taken(s)        |(x,y z)m in world frame|
|----------|----------------------|------------------|---------------------|-----------------------|
|1         |63                    |(29,64)           |0.011-0.030          |(5,2,1.8)              |
|2         |76                    |(398,64)          |0.020-0.040          |(5,1,1.8)              |
|3         |54                    |(465,64)          |0.025-0.040          |(5,0.8,1.8)            |

SIFT  
|Index     |No of Good matches    |Descriptor size   |time_taken(s)        |(x,y z)m in world frame|
|----------|----------------------|------------------|---------------------|-----------------------|
|1         |11                    |(22,128)          |0.054-0.075          |(5,2,1.8)
|2         |12                    |(95,128)          |0.055-0.070          |(5,1.5,1.8) 
|3         |12                    |(127,128)         |0.050-0.070          |(5,1,1.8)
|4         |12                    |(168,128)         |0.045-0.070          |(5,0.8,1.8)
