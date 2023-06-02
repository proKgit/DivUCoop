from asyncio import constants
from math import cos,sin
import h5py
import numpy as np
import open3d as o3d
import cv2
import csv
import numpy as np
import pcl.pcl_visualization
import cv2



FILE="/home/changhe/DivUCoop/data/R05D02_RC16/#R05D02_RC16_0530_ChengDu_2023-05-30_19-07-53_TDA4DDS.mf4_CDDS.sdf"

DSTPATH = "/home/changhe/DivUCoop/parsed/"

MOUNT_X=0
MOUNT_Y=0
CLU_STATE_GATE=0.8

ORINENTATION=1


CSV_PATH="/media/changhe/Data/SAIC_PR58307_H6_213_FRGen21R77/lidar_ts.csv"
PCD_PATH='/media/changhe/Data/SAIC_PR58307_H6_213_FRGen21R77/local_pcdbin'

def hdftimebase2ns(hdft):
    return int(hdft*1e4)

def writelist2txt(dstfilename,lst):
    with open(dstfilename, 'w') as file:
        line = ' '.join(str(item) for item in lst)
        file.write(line)

def parseSingeVdy(d,t):
    lst = d.tolist()
    fn =DSTPATH+"vdy/"+str(t)+".vdytxt"
    writelist2txt(fn,lst)
    
def parsevdy(vdydata,vdytime):
    for i in range(0,len(vdydata)):
        vd=vdydata[i][0]
        vt=hdftimebase2ns(vdytime[i][0])
        parseSingeVdy(vd,vt)


def parseSingeFreespace(fsd,fst):
    lst=[fsd[0],fsd[1]]
    ctrs=fsd[2]
    for pt in ctrs:
        for attr in pt:
            lst.append(attr)
    lst.extend([fsd[3],fsd[4]])
    fn =DSTPATH+"fs/"+str(fst)+".fstxt"
    writelist2txt(fn,lst)

def parsefreespace(fsdata,fstime):
    for i in range(0,len(fsdata)):
        fd=fsdata[i][0]
        ft=hdftimebase2ns(fstime[i][0])
        parseSingeFreespace(fd,ft)


def parseSingelObject(od,ot):
    lst = []
    for obj in od:
        for attr in obj:
            lst.append(attr)
    fn =DSTPATH+"obj/"+str(ot)+".objtxt"
    writelist2txt(fn,lst)


def parseobjlist(objdata,objt):
    for i in range(0,len(objdata)):
        od=objdata[i][0]
        ot=hdftimebase2ns(objt[i][0])
        parseSingelObject(od,ot)



def parseSinglePs(pd,pt):
    lst=[]
    for p in pd:
        for j in range(0,len(p)-1):
            lst.append(p[j])
        for num in p[-1]:
            lst.append(num)
    fn =DSTPATH+"ps/"+str(pt)+".pstxt"
    writelist2txt(fn,lst)

def parsepslist(psdata,pst):
    for i in range(0,len(psdata)):
        pd=psdata[i][0]
        pt=hdftimebase2ns(pst[i][0])
        parseSinglePs(pd,pt)

def parseSingleUss(ud,ut):
    lst=[]
    for num in ud:
        lst.append(num)
    fn =DSTPATH+"uss/"+str(ut)+".usstxt"
    writelist2txt(fn,lst)


def parseuss(ussdata,usst):
    for i in range(0,len(ussdata)):
        ud=ussdata[i][0]
        ut=hdftimebase2ns(usst[i][0])
        parseSingleUss(ud,ut)



f=h5py.File(FILE,'r')

print("VDY START")
vdy = f['Logdata']['Vehicle']['vehicle']['Data']
vdytime=f['Logdata']['Vehicle']['vehicle']['Time']
parsevdy(vdy,vdytime)



print("FS START")
freespace = f['Logdata']['SVS']['Freespace']['FSPP_CameraFreespaceType_t_Tag']['Data']
freespacetime = f['Logdata']['SVS']['Freespace']['FSPP_CameraFreespaceType_t_Tag']['Time']
parsefreespace(freespace,freespacetime)

print("OBJ START")
objlist = f['Logdata']['SVS']['Objs']['FSPP_CameraObjInput_t_Tag']['Data']['Obj']
objlisttime = f['Logdata']['SVS']['Objs']['FSPP_CameraObjInput_t_Tag']['Time']
parseobjlist(objlist,objlisttime)

print("PS START")
pslist = f['Logdata']['SVS']['ParkingSlot']['FSPP_CameraParkingSlotSet_t_Tag']['Data']['Slot_s']
pslisttime = f['Logdata']['SVS']['ParkingSlot']['FSPP_CameraParkingSlotSet_t_Tag']['Time']
parsepslist(pslist,pslisttime)

print("USS START")
usmeas = f['Logdata']['USS']['MinDistance']['FSPP_MinDistanceInput_t_Tag']['Data']['MinDistance']
usst = f['Logdata']['USS']['MinDistance']['FSPP_MinDistanceInput_t_Tag']['Time']
parseuss(usmeas,usst)




# return

# target=f['Logdata']['Radar_FC0']['Targets']['SigProcData']['Data']['RadarDetectionData']['Target_a']
# meas_length=target.shape[0]

# aln=f['Logdata']['Radar_FC0']['Targets']['TargetPerceptionDataType']['Data']['Payload']['Perception']['DiagData']['Misalignment']['Online']

# # timestamp_sigproc=f['Logdata']['Radar_FC0']['Targets']['SigProcData']['Time']
# context=f['Logdata']['Radar_FC0']['Targets']['SigProcData']['Data']['RadarDetectionData']['RadarContextData']

# # which ts is mapped to lidar label? ACQ time or meas time? shoule latency be considered?
# ts_generic=f['Logdata']['Radar_FC0']['Targets']['SigProcData']['Data']['RadarDetectionData']['GenericContextData']['TimestampMeasureEnd_u64']

# vdy=f['Logdata']['Radar_FC0']['Targets']['TargetPerceptionDataType']['Data']['Payload']['Perception']['HostDynData']

# clu_list=[]
# clu_list_color=[]


# vis = o3d.visualization.Visualizer()
# vis.create_window()

# pcd = o3d.geometry.PointCloud()
# lidarpcd=o3d.geometry.PointCloud()
# # vis.run()

# for i in range(0,meas_length):
#     print('Meas Counter ',i)

#     aln_az=aln[i,0]['Azimuth']['Value']
#     aln_elev=aln[i,0]['Elevation']['Value']
#     targetlist_current=target[i,0]
#     ts_current=ts_generic[i][0]              # in nanosec

#     pcd_name=findNearestPcd(ts_current*1e-9,lidar_ts)
#     print(pcd_name,ts_current*1e-9)
    
#     num_of_targets=context[i]["NumTargets_u16"][0]

#     res_current=context[i]["Resolution"]
#     res_current_range=context[i]["Resolution"]['Range_f32'][0]
#     res_current_velocity=context[i]["Resolution"]['Velocity_f32'][0]

#     ambig_current=context[i]['Ambiguity']
#     ambig_current_Range=context[i]['Ambiguity']['DistanceRange_f32'][0]
#     ambig_current_Velocity=context[i]['Ambiguity']['VelocityRange_f32'][0]
#     ambig_current_Az=context[i]['Ambiguity']['AzimuthRange_f32'][0]
#     ambig_current_Elev=context[i]['Ambiguity']['ElevationRange_f32'][0]

#     degradation_current=context[i]['SensorCapability']
#     degradation_current_jamming=context[i]['SensorCapability']['JammingStatus_u8'][0]
#     degradation_current_blockage=context[i]['SensorCapability']['BlockageStatus_u8'][0]



#     vx=vdy[i]['Speed']['Mean'][0]
#     ax=vdy[i]['AccelX']['Mean'][0]
#     ay=vdy[i]['AccelY']['Mean'][0]
#     yawrate=vdy[i]['YawRate']['Mean'][0]
#     pitchrate=vdy[i]['PitchRate']['Mean'][0]
#     vx_corr=vdy[i]['SpeedCorFac']['Value'][0]

#     vx=vx*vx_corr
#     clu_list=[]
#     clu_list_color=[]


#     # print(type(num_of_targets))
#     for j in range(0,num_of_targets):
#         # print('target ',j)
#         # print(type(j) == 'numpy.uint16')
        
#         t_meas=targetlist_current[j]['Measurement']

#         # if(t_meas['Range_f32'] < 0.0001):
            
#         #     continue
#         t_accuracy=targetlist_current[j]['MeasurementAccuracy']
#         t_qualifier=targetlist_current[j]['Qualifier']

#         t_meas['Range_f32']
#         t_meas['RadialVelocity_f32']
#         t_meas['Azimuth_f32'] - aln_az
#         t_meas['Elevation_f32'] - aln_elev
#         t_meas['RCS_f32']
#         t_meas['SNR_f32']
#         t_meas['Level_f32']   # -> meaning?

#         clu_range=np.asarray(t_meas['Range_f32'],np.float32)
#         clu_vrad=np.asarray(t_meas['RadialVelocity_f32'],np.float32)
#         clu_az=np.asarray(t_meas['Azimuth_f32'],np.float32)
#         clu_elev=np.asarray(t_meas['Elevation_f32'],np.float32)
#         clu_rcs=np.asarray(t_meas['RCS_f32'],np.float32)
        
#         if np.isnan(clu_range) or np.isnan(clu_vrad) or np.isnan(clu_az) or np.isnan(clu_elev) or np.isnan(clu_rcs):
#             continue


#         t_accuracy['RangeStd_f32']
#         t_accuracy['RadialVelocityStd_f32']
#         t_accuracy['AzimuthStd_f32']
#         t_accuracy['ElevationStd_f32']

#         t_qualifier['FalseDetectionProb_f32'] 
#         t_qualifier['ModelDeviation']['Range_f32']
#         t_qualifier['ModelDeviation']['RadialVelocity_f32']
#         t_qualifier['ModelDeviation']['Azimuth_f32']
#         t_qualifier['ModelDeviation']['Elevation_f32']
#         t_qualifier['ModelDeviation']['AngularAmbiguity_f32']
#         t_qualifier['QualifierFlags']
#         pt_cart=polar2cat(clu_range,clu_az,clu_elev,clu_vrad)
#         if(pt_cart[2]< -0.75):
#             continue

#         clu_list.append([pt_cart[0],pt_cart[1],pt_cart[2]])
#         if isMoving(vx,yawrate,clu_az,clu_elev,clu_vrad,ambig_current_Velocity):
#             clu_list_color.append([255,0,0])
#         else:
#             clu_list_color.append([0,0,0])
    
#     clu_list_np=np.array(clu_list)
#     clu_list_color_np=np.array(clu_list_color)





#     # pcd.points = o3d.utility.Vector3dVector(clu_list_np)
#     # pcd.colors=o3d.utility.Vector3dVector(clu_list_color_np)
#     # pcd.translate((-3.72,0,-0.75),relative=True)

#     # lidar_pcd = o3d.io.read_point_cloud(PCD_PATH+'/'+pcd_name)
#     # # lidar_pcd.paint_uniform_color([1, 0.706, 0])
#     # # o3d.visualization.draw_geometries([pcd,lidar_pcd])

#     # # vis.update_geometry(pcd)
#     # vis.add_geometry(pcd)
#     # vis.add_geometry(lidar_pcd)
#     # vis.poll_events()
#     # vis.update_renderer()
#     # vis.remove_geometry(lidar_pcd)
#     # cv2.imshow("temp",img)
#     # cv2.waitKey()

#     # vis.run()

