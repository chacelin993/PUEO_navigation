import pandas as pd
import numpy as np
import dataframe_image as dfi
folder_name="Sep13/";
table_h=np.genfromtxt("{}table_h.txt".format(folder_name),delimiter=',');
table_p=np.genfromtxt("{}table_p.txt".format(folder_name),delimiter=',');
table_r=np.genfromtxt("{}table_r.txt".format(folder_name),delimiter=',');
table_hpr=np.array([table_h,table_p,table_r]);
main_header= np.array(['','Fast','','','Slow','']);
column_header=np.array(['Average','Error in Average','Standard Deviation',\
                        'Average','Error in Average','Standard Deviation']);
header=[main_header,column_header];
filter_setting=np.array(['Adaptive','Adaptive','Automobile','Walking']);
image_name=["{}table_h.png".format(folder_name),"{}table_p.png".format(folder_name),"{}table_r.png".format(folder_name)];
for i in range(0,3):
    df = pd.DataFrame(table_hpr[i,:,:],index=filter_setting,columns=header)
    pd.set_option('display.float_format', '{:.6g}'.format)
    dfi.export(df,image_name[i] );
