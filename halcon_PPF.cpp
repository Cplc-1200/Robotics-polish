#include "halcon_PPF.h"
void Halcon_PPFMatch(HTuple* hv_Pose, HTuple* hv_Score, HTuple* hv_Hom3d)
{
    // Local control variables
    HTuple  hv_ObjectModel3D, hv_Status, hv_ObjectModel3DNormals;
    HTuple  hv_Diameter, hv_Model3DSampled, hv_SurfaceModelID, ObjectModel3DRigidTrans;
    HTuple  hv_Scene3D, hv_Model3DScene3D, hv_NotUsed, hv_SurfaceMatchingResultID;

    //dev_get_window (WindowHandle)
    //读取3D模型

    ReadObjectModel3d("model_file/delau_mesh.ply",
        "mm", "convert_to_triangles", "true", &hv_ObjectModel3D, &hv_Status);
    //Title := 'Reference object (uncalibrated measurement)'
    //Instructions[0] := 'Rotate: Left button'
    //Instructions[1] := 'Zoom:   Shift + left button'
    //Instructions[2] := 'Move:   Ctrl  + left button'
    //显示3D模型
    //visualize_object_model_3d (WindowHandle, ObjectModel3D, [], [], [], [], Title, [], Instructions, PoseOut)

    //计算三维对象模型的三维曲面法线
    SurfaceNormalsObjectModel3d(hv_ObjectModel3D, "mls", "mls_force_inwards", "true",
        &hv_ObjectModel3DNormals);
    //3D模型的最大直径
    MaxDiameterObjectModel3d(hv_ObjectModel3DNormals, &hv_Diameter);
    //对3D模型采样
    SampleObjectModel3d(hv_ObjectModel3DNormals, "fast", hv_Diameter * 0.006, HTuple(),
        HTuple(), &hv_Model3DSampled);
    //visualize_object_model_3d (WindowHandle, Model3DSampled, [], [], [], [], Title, [], Instructions, PoseOut)
    //创建基于表面匹配所需的数据结构。
    CreateSurfaceModel(hv_ObjectModel3DNormals, 0.03, "model_invert_normals", "true",
        &hv_SurfaceModelID);

    ReadObjectModel3d("model_file/post_camera_cloud.ply",
        "mm", "convert_to_triangles", "true", &hv_Scene3D, &hv_Status);
    //triangulate_object_model_3d (Scene3D, 'greedy', [], [], TriangulatedObjectModel3D, Information)
    SurfaceNormalsObjectModel3d(hv_Scene3D, "mls", HTuple(), HTuple(), &hv_Model3DScene3D);

    FindSurfaceModel(hv_SurfaceModelID, hv_Model3DScene3D, 0.02, 0.2, 0.3, "false",
        HTuple(), HTuple(), &(*hv_Pose), &(*hv_Score), &hv_NotUsed);
    RefineSurfaceModelPose(hv_SurfaceModelID, hv_Model3DScene3D, (*hv_Pose), 0, "false",
        HTuple(), HTuple(), &(*hv_Pose), &(*hv_Score), &hv_SurfaceMatchingResultID);
    PoseToHomMat3d(*hv_Pose, &(*hv_Hom3d));
    //rigid_trans_object_model_3d (Model3DSampled, Pose, ObjectModel3DRigidTrans)
   // RigidTransObjectModel3d(hv_Model3DSampled, *hv_Score, &ObjectModel3DRigidTrans);
    //visualize_object_model_3d (WindowHandle, [ObjectModel3DRigidTrans,Model3DScene3D], [], [], ['color_0','color_1'], ['green','red'], [], [], [], PoseOut11)
    
    //visualize_object_model_3d (WindowHandle, [SurfaceMatchingResultID, Model3DScene3D], [], [], ['lut','color_attrib','disp_pose'], ['color1','coord_z','true'], [], [], [], PoseOut)
}