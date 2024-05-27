QMAKE_CXXFLAGS += /arch:AVX
QT       += core gui serialbus network concurrent openglwidgets
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
DEFINES += QT_DEPRECATED_WARNINGS

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    mainwindow.h

FORMS += \
    mainwindow.ui

#pragma region
#--------------------------------------------PCL dependence-------------------------------------------

INCLUDEPATH += ../PCL1.13.0/include/pcl-1.13\

INCLUDEPATH += ../PCL1.13.0/include/pcl-1.13/pcl\

INCLUDEPATH += ../PCL1.13.0/3rdParty/Boost/include/boost-1_80\


INCLUDEPATH += ../PCL1.13.0/3rdParty/Eigen/eigen3\


INCLUDEPATH += ../PCL1.13.0/3rdParty/FLANN/include\


INCLUDEPATH += ../PCL1.13.0/3rdParty/FLANN/include/flann\


INCLUDEPATH += ../PCL1.13.0/3rdParty/OpenNI2/Include\


INCLUDEPATH += ../PCL1.13.0/3rdParty/Qhull/include\


INCLUDEPATH += ../PCL1.13.0/3rdParty/VTK/include/vtk-9.2\

CONFIG(debug,debug|release){

LIBS += -L../PCL1.13.0/lib\
        -lpcl_commond\
        -lpcl_featuresd\
        -lpcl_filtersd\
        -lpcl_iod\
        -lpcl_io_plyd\
        -lpcl_kdtreed\
        -lpcl_keypointsd\
        -lpcl_mld\
        -lpcl_octreed\
        -lpcl_outofcored\
        -lpcl_peopled\
        -lpcl_recognitiond\
        -lpcl_registrationd\
        -lpcl_sample_consensusd\
        -lpcl_searchd\
        -lpcl_segmentationd\
        -lpcl_stereod\
        -lpcl_surfaced\
        -lpcl_trackingd\
        -lpcl_visualizationd\

LIBS += -L../PCL1.13.0/3rdParty/Boost/lib\
       -llibboost_atomic-vc143-mt-gd-x64-1_80\
       -llibboost_bzip2-vc143-mt-gd-x64-1_80\
       -llibboost_chrono-vc143-mt-gd-x64-1_80\
       -llibboost_container-vc143-mt-gd-x64-1_80\
       -llibboost_context-vc143-mt-gd-x64-1_80\
       -llibboost_contract-vc143-mt-gd-x64-1_80\
       -llibboost_coroutine-vc143-mt-gd-x64-1_80\
       -llibboost_date_time-vc143-mt-gd-x64-1_80\
       -llibboost_exception-vc143-mt-gd-x64-1_80\
       -llibboost_fiber-vc143-mt-gd-x64-1_80\
       -llibboost_filesystem-vc143-mt-gd-x64-1_80\
       -llibboost_graph-vc143-mt-gd-x64-1_80\
       -llibboost_graph_parallel-vc143-mt-gd-x64-1_80\
       -llibboost_iostreams-vc143-mt-gd-x64-1_80\
       -llibboost_json-vc143-mt-gd-x64-1_80\
       -llibboost_locale-vc143-mt-gd-x64-1_80\
       -llibboost_log-vc143-mt-gd-x64-1_80\
       -llibboost_log_setup-vc143-mt-gd-x64-1_80\
       -llibboost_math_c99-vc143-mt-gd-x64-1_80\
       -llibboost_math_c99f-vc143-mt-gd-x64-1_80\
       -llibboost_math_c99l-vc143-mt-gd-x64-1_80\
       -llibboost_math_tr1-vc143-mt-gd-x64-1_80\
       -llibboost_math_tr1f-vc143-mt-gd-x64-1_80\
       -llibboost_math_tr1l-vc143-mt-gd-x64-1_80\
       -llibboost_mpi-vc143-mt-gd-x64-1_80\
       -llibboost_nowide-vc143-mt-gd-x64-1_80\
       -llibboost_numpy310-vc143-mt-gd-x64-1_80\
       -llibboost_prg_exec_monitor-vc143-mt-gd-x64-1_80\
       -llibboost_program_options-vc143-mt-gd-x64-1_80\
       -llibboost_python310-vc143-mt-gd-x64-1_80\
       -llibboost_random-vc143-mt-gd-x64-1_80\
       -llibboost_regex-vc143-mt-gd-x64-1_80\
       -llibboost_serialization-vc143-mt-gd-x64-1_80\
       -llibboost_stacktrace_noop-vc143-mt-gd-x64-1_80\
       -llibboost_stacktrace_windbg-vc143-mt-gd-x64-1_80\
       -llibboost_stacktrace_windbg_cached-vc143-mt-gd-x64-1_80\
       -llibboost_system-vc143-mt-gd-x64-1_80\
       -llibboost_test_exec_monitor-vc143-mt-gd-x64-1_80\
       -llibboost_thread-vc143-mt-gd-x64-1_80\
       -llibboost_timer-vc143-mt-gd-x64-1_80\
       -llibboost_type_erasure-vc143-mt-gd-x64-1_80\
       -llibboost_unit_test_framework-vc143-mt-gd-x64-1_80\
       -llibboost_wave-vc143-mt-gd-x64-1_80\
       -llibboost_wserialization-vc143-mt-gd-x64-1_80\
       -llibboost_zlib-vc143-mt-gd-x64-1_80\




LIBS += -L../PCL1.13.0/3rdParty/FLANN/lib\
        -lflann-gd\
        -lflann_cpp-gd\
        -lflann_cpp_s-gd\
        -lflann_s-gd\

LIBS += -L../PCL1.13.0/3rdParty/OpenNI2/Lib\
        -lOpenNI2

LIBS += -L../PCL1.13.0/3rdParty/Qhull/lib\
        -lqhullcpp_d\
        -lqhullstatic_d\
        -lqhullstatic_rd\
        -lqhull_rd\

LIBS += -L../PCL1.13.0/3rdParty/VTK/lib\
          -lvtkcgns-9.2d\
          -lvtkChartsCore-9.2d\
          -lvtkCommonColor-9.2d\
          -lvtkCommonComputationalGeometry-9.2d\
          -lvtkCommonCore-9.2d\
          -lvtkCommonDataModel-9.2d\
          -lvtkCommonExecutionModel-9.2d\
          -lvtkCommonMath-9.2d\
          -lvtkCommonMisc-9.2d\
          -lvtkCommonSystem-9.2d\
          -lvtkCommonTransforms-9.2d\
          -lvtkDICOMParser-9.2d\
          -lvtkDomainsChemistry-9.2d\
          -lvtkDomainsChemistryOpenGL2-9.2d\
          -lvtkdoubleconversion-9.2d\
          -lvtkexodusII-9.2d\
          -lvtkexpat-9.2d\
          -lvtkFiltersAMR-9.2d\
          -lvtkFiltersCore-9.2d\
          -lvtkFiltersExtraction-9.2d\
          -lvtkFiltersFlowPaths-9.2d\
          -lvtkFiltersGeneral-9.2d\
          -lvtkFiltersGeneric-9.2d\
          -lvtkFiltersGeometry-9.2d\
          -lvtkFiltersHybrid-9.2d\
          -lvtkFiltersHyperTree-9.2d\
          -lvtkFiltersImaging-9.2d\
          -lvtkFiltersModeling-9.2d\
          -lvtkFiltersParallel-9.2d\
          -lvtkFiltersParallelImaging-9.2d\
          -lvtkFiltersPoints-9.2d\
          -lvtkFiltersProgrammable-9.2d\
          -lvtkFiltersSelection-9.2d\
          -lvtkFiltersSMP-9.2d\
          -lvtkFiltersSources-9.2d\
          -lvtkFiltersStatistics-9.2d\
          -lvtkFiltersTexture-9.2d\
          -lvtkFiltersTopology-9.2d\
          -lvtkFiltersVerdict-9.2d\
          -lvtkfmt-9.2d\
          -lvtkfreetype-9.2d\
          -lvtkGeovisCore-9.2d\
          -lvtkgl2ps-9.2d\
          -lvtkglew-9.2d\
          -lvtkGUISupportQt-9.2d\
          -lvtkGUISupportQtQuick-9.2d\
          -lvtkGUISupportQtSQL-9.2d\
          -lvtkhdf5-9.2d\
          -lvtkhdf5_hl-9.2d\
          -lvtkImagingColor-9.2d\
          -lvtkImagingCore-9.2d\
          -lvtkImagingFourier-9.2d\
          -lvtkImagingGeneral-9.2d\
          -lvtkImagingHybrid-9.2d\
          -lvtkImagingMath-9.2d\
          -lvtkImagingMorphological-9.2d\
          -lvtkImagingSources-9.2d\
          -lvtkImagingStatistics-9.2d\
          -lvtkImagingStencil-9.2d\
          -lvtkInfovisCore-9.2d\
          -lvtkInfovisLayout-9.2d\
          -lvtkInteractionImage-9.2d\
          -lvtkInteractionStyle-9.2d\
          -lvtkInteractionWidgets-9.2d\
          -lvtkIOAMR-9.2d\
          -lvtkIOAsynchronous-9.2d\
          -lvtkIOCesium3DTiles-9.2d\
          -lvtkIOCGNSReader-9.2d\
          -lvtkIOChemistry-9.2d\
          -lvtkIOCityGML-9.2d\
          -lvtkIOCONVERGECFD-9.2d\
          -lvtkIOCore-9.2d\
          -lvtkIOEnSight-9.2d\
          -lvtkIOExodus-9.2d\
          -lvtkIOExport-9.2d\
          -lvtkIOExportGL2PS-9.2d\
          -lvtkIOExportPDF-9.2d\
          -lvtkIOGeometry-9.2d\
          -lvtkIOHDF-9.2d\
          -lvtkIOImage-9.2d\
          -lvtkIOImport-9.2d\
          -lvtkIOInfovis-9.2d\
          -lvtkIOIOSS-9.2d\
          -lvtkIOLegacy-9.2d\
          -lvtkIOLSDyna-9.2d\
          -lvtkIOMINC-9.2d\
          -lvtkIOMotionFX-9.2d\
          -lvtkIOMovie-9.2d\
          -lvtkIONetCDF-9.2d\
          -lvtkIOOggTheora-9.2d\
          -lvtkIOParallel-9.2d\
          -lvtkIOParallelXML-9.2d\
          -lvtkIOPLY-9.2d\
          -lvtkIOSegY-9.2d\
          -lvtkIOSQL-9.2d\
          -lvtkioss-9.2d\
          -lvtkIOTecplotTable-9.2d\
          -lvtkIOVeraOut-9.2d\
          -lvtkIOVideo-9.2d\
          -lvtkIOXML-9.2d\
          -lvtkIOXMLParser-9.2d\
          -lvtkjpeg-9.2d\
          -lvtkjsoncpp-9.2d\
          -lvtkkissfft-9.2d\
          -lvtklibharu-9.2d\
          -lvtklibproj-9.2d\
          -lvtklibxml2-9.2d\
          -lvtkloguru-9.2d\
          -lvtklz4-9.2d\
          -lvtklzma-9.2d\
          -lvtkmetaio-9.2d\
          -lvtknetcdf-9.2d\
          -lvtkogg-9.2d\
          -lvtkParallelCore-9.2d\
          -lvtkParallelDIY-9.2d\
          -lvtkpng-9.2d\
          -lvtkpugixml-9.2d\
          -lvtkRenderingAnnotation-9.2d\
          -lvtkRenderingContext2D-9.2d\
          -lvtkRenderingContextOpenGL2-9.2d\
          -lvtkRenderingCore-9.2d\
          -lvtkRenderingFreeType-9.2d\
          -lvtkRenderingGL2PSOpenGL2-9.2d\
          -lvtkRenderingHyperTreeGrid-9.2d\
          -lvtkRenderingImage-9.2d\
          -lvtkRenderingLabel-9.2d\
          -lvtkRenderingLICOpenGL2-9.2d\
          -lvtkRenderingLOD-9.2d\
          -lvtkRenderingOpenGL2-9.2d\
          -lvtkRenderingQt-9.2d\
          -lvtkRenderingSceneGraph-9.2d\
          -lvtkRenderingUI-9.2d\
          -lvtkRenderingVolume-9.2d\
          -lvtkRenderingVolumeOpenGL2-9.2d\
          -lvtkRenderingVtkJS-9.2d\
          -lvtksqlite-9.2d\
          -lvtksys-9.2d\
          -lvtkTestingRendering-9.2d\
          -lvtktheora-9.2d\
          -lvtktiff-9.2d\
          -lvtkverdict-9.2d\
          -lvtkViewsContext2D-9.2d\
          -lvtkViewsCore-9.2d\
          -lvtkViewsInfovis-9.2d\
          -lvtkViewsQt-9.2d\
          -lvtkWrappingTools-9.2d\
          -lvtkzlib-9.2d\


}
else {

LIBS += -L../PCL1.13.0/lib\
        -lpcl_common\
        -lpcl_features\
        -lpcl_filters\
        -lpcl_io\
        -lpcl_io_ply\
        -lpcl_kdtree\
        -lpcl_keypoints\
        -lpcl_ml\
        -lpcl_octree\
        -lpcl_outofcore\
        -lpcl_people\
        -lpcl_recognition\
        -lpcl_registration\
        -lpcl_sample_consensus\
        -lpcl_search\
        -lpcl_segmentation\
        -lpcl_stereo\
        -lpcl_surface\
        -lpcl_tracking\
        -lpcl_visualization\

LIBS += -L../PCL1.13.0/3rdParty/Boost/lib\
      -llibboost_atomic-vc143-mt-x64-1_80\
      -llibboost_bzip2-vc143-mt-x64-1_80\
      -llibboost_chrono-vc143-mt-x64-1_80\
      -llibboost_container-vc143-mt-x64-1_80\
      -llibboost_context-vc143-mt-x64-1_80\
      -llibboost_contract-vc143-mt-x64-1_80\
      -llibboost_coroutine-vc143-mt-x64-1_80\
      -llibboost_date_time-vc143-mt-x64-1_80\
      -llibboost_exception-vc143-mt-x64-1_80\
      -llibboost_fiber-vc143-mt-x64-1_80\
      -llibboost_filesystem-vc143-mt-x64-1_80\
      -llibboost_graph-vc143-mt-x64-1_80\
      -llibboost_graph_parallel-vc143-mt-x64-1_80\
      -llibboost_iostreams-vc143-mt-x64-1_80\
      -llibboost_json-vc143-mt-x64-1_80\
      -llibboost_locale-vc143-mt-x64-1_80\
      -llibboost_log-vc143-mt-x64-1_80\
      -llibboost_log_setup-vc143-mt-x64-1_80\
      -llibboost_math_c99-vc143-mt-x64-1_80\
      -llibboost_math_c99f-vc143-mt-x64-1_80\
      -llibboost_math_c99l-vc143-mt-x64-1_80\
      -llibboost_math_tr1-vc143-mt-x64-1_80\
      -llibboost_math_tr1f-vc143-mt-x64-1_80\
      -llibboost_math_tr1l-vc143-mt-x64-1_80\
      -llibboost_mpi-vc143-mt-x64-1_80\
      -llibboost_nowide-vc143-mt-x64-1_80\
      -llibboost_numpy310-vc143-mt-x64-1_80\
      -llibboost_prg_exec_monitor-vc143-mt-x64-1_80\
      -llibboost_program_options-vc143-mt-x64-1_80\
      -llibboost_python310-vc143-mt-x64-1_80\
      -llibboost_random-vc143-mt-x64-1_80\
      -llibboost_regex-vc143-mt-x64-1_80\
      -llibboost_serialization-vc143-mt-x64-1_80\
      -llibboost_stacktrace_noop-vc143-mt-x64-1_80\
      -llibboost_stacktrace_windbg-vc143-mt-x64-1_80\
      -llibboost_stacktrace_windbg_cached-vc143-mt-x64-1_80\
      -llibboost_system-vc143-mt-x64-1_80\
      -llibboost_test_exec_monitor-vc143-mt-x64-1_80\
      -llibboost_thread-vc143-mt-x64-1_80\
      -llibboost_timer-vc143-mt-x64-1_80\
      -llibboost_type_erasure-vc143-mt-x64-1_80\
      -llibboost_unit_test_framework-vc143-mt-x64-1_80\
      -llibboost_wave-vc143-mt-x64-1_80\
      -llibboost_wserialization-vc143-mt-x64-1_80\
      -llibboost_zlib-vc143-mt-x64-1_80\



LIBS += -L../PCL1.13.0/3rdParty/FLANN/lib\
        -lflann\
        -lflann_cpp\
        -lflann_cpp_s\
        -lflann_s

LIBS += -L../PCL1.13.0/3rdParty/OpenNI2/Lib\
        -lOpenNI2

LIBS += -L../PCL1.13.0/3rdParty/Qhull/lib\
        -lqhullcpp\
        -lqhullstatic\
        -lqhullstatic_r\
        -lqhull_r\

LIBS += -L../PCL1.13.0/3rdParty/VTK/lib\
       -lvtkcgns-9.2\
       -lvtkChartsCore-9.2\
       -lvtkCommonColor-9.2\
       -lvtkCommonComputationalGeometry-9.2\
       -lvtkCommonCore-9.2\
       -lvtkCommonDataModel-9.2\
       -lvtkCommonExecutionModel-9.2\
       -lvtkCommonMath-9.2\
       -lvtkCommonMisc-9.2\
       -lvtkCommonSystem-9.2\
       -lvtkCommonTransforms-9.2\
       -lvtkDICOMParser-9.2\
       -lvtkDomainsChemistry-9.2\
       -lvtkDomainsChemistryOpenGL2-9.2\
       -lvtkdoubleconversion-9.2\
       -lvtkexodusII-9.2\
       -lvtkexpat-9.2\
       -lvtkFiltersAMR-9.2\
       -lvtkFiltersCore-9.2\
       -lvtkFiltersExtraction-9.2\
       -lvtkFiltersFlowPaths-9.2\
       -lvtkFiltersGeneral-9.2\
       -lvtkFiltersGeneric-9.2\
       -lvtkFiltersGeometry-9.2\
       -lvtkFiltersHybrid-9.2\
       -lvtkFiltersHyperTree-9.2\
       -lvtkFiltersImaging-9.2\
       -lvtkFiltersModeling-9.2\
       -lvtkFiltersParallel-9.2\
       -lvtkFiltersParallelImaging-9.2\
       -lvtkFiltersPoints-9.2\
       -lvtkFiltersProgrammable-9.2\
       -lvtkFiltersSelection-9.2\
       -lvtkFiltersSMP-9.2\
       -lvtkFiltersSources-9.2\
       -lvtkFiltersStatistics-9.2\
       -lvtkFiltersTexture-9.2\
       -lvtkFiltersTopology-9.2\
       -lvtkFiltersVerdict-9.2\
       -lvtkfmt-9.2\
       -lvtkfreetype-9.2\
       -lvtkGeovisCore-9.2\
       -lvtkgl2ps-9.2\
       -lvtkglew-9.2\
       -lvtkGUISupportQt-9.2\
       -lvtkGUISupportQtQuick-9.2\
       -lvtkGUISupportQtSQL-9.2\
       -lvtkhdf5-9.2\
       -lvtkhdf5_hl-9.2\
       -lvtkImagingColor-9.2\
       -lvtkImagingCore-9.2\
       -lvtkImagingFourier-9.2\
       -lvtkImagingGeneral-9.2\
       -lvtkImagingHybrid-9.2\
       -lvtkImagingMath-9.2\
       -lvtkImagingMorphological-9.2\
       -lvtkImagingSources-9.2\
       -lvtkImagingStatistics-9.2\
       -lvtkImagingStencil-9.2\
       -lvtkInfovisCore-9.2\
       -lvtkInfovisLayout-9.2\
       -lvtkInteractionImage-9.2\
       -lvtkInteractionStyle-9.2\
       -lvtkInteractionWidgets-9.2\
       -lvtkIOAMR-9.2\
       -lvtkIOAsynchronous-9.2\
       -lvtkIOCesium3DTiles-9.2\
       -lvtkIOCGNSReader-9.2\
       -lvtkIOChemistry-9.2\
       -lvtkIOCityGML-9.2\
       -lvtkIOCONVERGECFD-9.2\
       -lvtkIOCore-9.2\
       -lvtkIOEnSight-9.2\
       -lvtkIOExodus-9.2\
       -lvtkIOExport-9.2\
       -lvtkIOExportGL2PS-9.2\
       -lvtkIOExportPDF-9.2\
       -lvtkIOGeometry-9.2\
       -lvtkIOHDF-9.2\
       -lvtkIOImage-9.2\
       -lvtkIOImport-9.2\
       -lvtkIOInfovis-9.2\
       -lvtkIOIOSS-9.2\
       -lvtkIOLegacy-9.2\
       -lvtkIOLSDyna-9.2\
       -lvtkIOMINC-9.2\
       -lvtkIOMotionFX-9.2\
       -lvtkIOMovie-9.2\
       -lvtkIONetCDF-9.2\
       -lvtkIOOggTheora-9.2\
       -lvtkIOParallel-9.2\
       -lvtkIOParallelXML-9.2\
       -lvtkIOPLY-9.2\
       -lvtkIOSegY-9.2\
       -lvtkIOSQL-9.2\
       -lvtkioss-9.2\
       -lvtkIOTecplotTable-9.2\
       -lvtkIOVeraOut-9.2\
       -lvtkIOVideo-9.2\
       -lvtkIOXML-9.2\
       -lvtkIOXMLParser-9.2\
       -lvtkjpeg-9.2\
       -lvtkjsoncpp-9.2\
       -lvtkkissfft-9.2\
       -lvtklibharu-9.2\
       -lvtklibproj-9.2\
       -lvtklibxml2-9.2\
       -lvtkloguru-9.2\
       -lvtklz4-9.2\
       -lvtklzma-9.2\
       -lvtkmetaio-9.2\
       -lvtknetcdf-9.2\
       -lvtkogg-9.2\
       -lvtkParallelCore-9.2\
       -lvtkParallelDIY-9.2\
       -lvtkpng-9.2\
       -lvtkpugixml-9.2\
       -lvtkRenderingAnnotation-9.2\
       -lvtkRenderingContext2D-9.2\
       -lvtkRenderingContextOpenGL2-9.2\
       -lvtkRenderingCore-9.2\
       -lvtkRenderingFreeType-9.2\
       -lvtkRenderingGL2PSOpenGL2-9.2\
       -lvtkRenderingHyperTreeGrid-9.2\
       -lvtkRenderingImage-9.2\
       -lvtkRenderingLabel-9.2\
       -lvtkRenderingLICOpenGL2-9.2\
       -lvtkRenderingLOD-9.2\
       -lvtkRenderingOpenGL2-9.2\
       -lvtkRenderingQt-9.2\
       -lvtkRenderingSceneGraph-9.2\
       -lvtkRenderingUI-9.2\
       -lvtkRenderingVolume-9.2\
       -lvtkRenderingVolumeOpenGL2-9.2\
       -lvtkRenderingVtkJS-9.2\
       -lvtksqlite-9.2\
       -lvtksys-9.2\
       -lvtkTestingRendering-9.2\
       -lvtktheora-9.2\
       -lvtktiff-9.2\
       -lvtkverdict-9.2\
       -lvtkViewsContext2D-9.2\
       -lvtkViewsCore-9.2\
       -lvtkViewsInfovis-9.2\
       -lvtkViewsQt-9.2\
       -lvtkWrappingTools-9.2\
       -lvtkzlib-9.2\
}
#pragma endregion
# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
