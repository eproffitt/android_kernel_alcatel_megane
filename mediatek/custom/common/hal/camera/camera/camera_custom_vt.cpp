
#include "camera_custom_vt.h"


NSCamCustom::SensorOrientation_T const& get_VTOrientation(void)
{

    static NSCamCustom::SensorOrientation_T const inst = {
        u4Degree_0  : 90,   //  main sensor in degree (0, 90, 180, 270)
        u4Degree_1  : 270,    //  sub  sensor in degree (0, 90, 180, 270)
    };
    return inst;

}

