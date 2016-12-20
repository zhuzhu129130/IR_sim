//--------------------------------------------------------------------------------------
//Data:    		20160902
//Purpose:  	3D Simulation for IR Scene
//Author:   	xiaozhu
//--------------------------------------------------------------------------------------

#include "Graphics_IRrad.h"

Graphics_IRrad::Graphics_IRrad()
{

}

//--------------------------------------------------------------------------------------
//=====================几何一致性数学计算公式==================================
void Graphics_IRrad::ScrRot(SCRPOS * sp, SCRPOS * opt, SCRPOS * ori, float32 theta)
{//sp为结果坐标，ori为屏幕待旋转矢量的出发点，opt为屏幕待旋转矢量的结束点，theta为旋转角度
    //由(opt-ori)旋转theta角度到达(sp-ori)
    sp->x =  (int32)((opt->x-ori->x)*cos(theta)-(opt->y-ori->y)*sin(theta)+ori->x);
    sp->y =  (int32)((opt->x-ori->x)*sin(theta)+(opt->y-ori->y)*cos(theta)+ori->y);
}

// C3DGraphicsMath message handlers
//--------------------------------------------------------------------------------------
VECTOR3 Graphics_IRrad::Vec3SetValue(VECTOR3 * vec, float32 x, float32 y, float32 z)
{//设置三维矢量值
    vec->x = x;
    vec->y = y;
    vec->z = z;
    return *vec;
}
//--------------------------------------------------------------------------------------
VECTOR3 Graphics_IRrad::Vec3Add(VECTOR3 * vec, VECTOR3 * vec1, VECTOR3 * vec2)//二个三维矢量相加
{
    vec->x = vec1->x + vec2->x;
    vec->y = vec1->y + vec2->y;
    vec->z = vec1->z + vec2->z;
    return *vec;
}
//--------------------------------------------------------------------------------------
VECTOR3 Graphics_IRrad::Vec3Subtract(VECTOR3 * vec, VECTOR3 * vec1, VECTOR3 * vec2)//二个三维矢量相减
{
    vec->x = vec1->x - vec2->x;
    vec->y = vec1->y - vec2->y;
    vec->z = vec1->z - vec2->z;
    return *vec;

}
//--------------------------------------------------------------------------------------
float32 Graphics_IRrad::Vec3GetLength(float32 * len, VECTOR3 * vec)//三维矢量求模
{
    *len = sqrt( vec->x * vec->x + vec->y * vec->y + vec->z * vec->z );
    return *len;
}
//--------------------------------------------------------------------------------------
float32 Graphics_IRrad::Vec4GetLength(float32 * len, VECTOR4 * vec)//四维矢量求模
{
    *len = sqrt( vec->q1 * vec->q1 + vec->q2 * vec->q2 + vec->q3 * vec->q3 + vec->q4 * vec->q4 );
    return *len;
}
//--------------------------------------------------------------------------------------
VECTOR3 Graphics_IRrad::Vec3Normalize(VECTOR3 * vec, VECTOR3 * vec1)//对三维矢量归一化
{
    float32 tmp;
    tmp = (Vec3GetLength(&tmp, vec1)==0.0f? 1.0f: tmp);//防止除数为零
    tmp = 1.0f/tmp;
    vec->x = vec1->x * tmp;
    vec->y = vec1->y * tmp;
    vec->z = vec1->z * tmp;
    return *vec;
}
//--------------------------------------------------------------------------------------
VECTOR3 Graphics_IRrad::Vec3Cross(VECTOR3 * vec, VECTOR3 * vec1, VECTOR3 * vec2)//二个三维矢量叉乘
{
    vec->x = vec1->y * vec2->z - vec1->z * vec2->y;
    vec->y = vec1->z * vec2->x - vec1->x * vec2->z;
    vec->z = vec1->x * vec2->y - vec1->y * vec2->x;
    return *vec;
}
//--------------------------------------------------------------------------------------
float32 Graphics_IRrad::Vec3Dot(float32 * result, VECTOR3 * vec1, VECTOR3 * vec2)//二个三维矢量点乘
{
    *result = vec1->x * vec2->x + vec1->y * vec2->y + vec1->z * vec2->z;
    return *result;
}

//--------------------------------------------------------------------------------------
VECTOR3 Graphics_IRrad::Vec3MultiConstant(VECTOR3 * vec, VECTOR3 * vec1, float32 num)//矢量乘以常数
{
    vec->x = vec1->x * num;
    vec->y = vec1->y * num;
    vec->z = vec1->z * num;
    return *vec;
}

//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
MATRIX16 Graphics_IRrad::Mat16SetValue(MATRIX16 * mat, float32 _11, float32 _12, float32 _13, float32 _14,
                                   float32 _21, float32 _22, float32 _23, float32 _24,
                                   float32 _31, float32 _32, float32 _33, float32 _34,
                                   float32 _41, float32 _42, float32 _43, float32 _44)
{//设置一个4*4矩阵的值
    mat->_11 = _11;
    mat->_12 = _12;
    mat->_13 = _13;
    mat->_14 = _14;
    mat->_21 = _21;
    mat->_22 = _22;
    mat->_23 = _23;
    mat->_24 = _24;
    mat->_31 = _31;
    mat->_32 = _32;
    mat->_33 = _33;
    mat->_34 = _34;
    mat->_41 = _41;
    mat->_42 = _42;
    mat->_43 = _43;
    mat->_44 = _44;
    return *mat;
}
//--------------------------------------------------------------------------------------
MATRIX16 Graphics_IRrad::Mat16SetIdentity(MATRIX16 * mat)//设置一个4*4的单位矩阵
{
    Mat16SetValue(mat, 1.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 1.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 1.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f);
    return *mat;
}
//--------------------------------------------------------------------------------------
MATRIX16 Graphics_IRrad::MatrixMultiplyMatrix(MATRIX16 * outmat, MATRIX16 * mat1, MATRIX16 * mat2)
{//二个4*4的矩阵相乘，结果作为输出返回值
    outmat->_11 = mat1->_11*mat2->_11 + mat1->_12*mat2->_21 + mat1->_13*mat2->_31 + mat1->_14*mat2->_41;
    outmat->_12 = mat1->_11*mat2->_12 + mat1->_12*mat2->_22 + mat1->_13*mat2->_32 + mat1->_14*mat2->_42;
    outmat->_13 = mat1->_11*mat2->_13 + mat1->_12*mat2->_23 + mat1->_13*mat2->_33 + mat1->_14*mat2->_43;
    outmat->_14 = mat1->_11*mat2->_14 + mat1->_12*mat2->_24 + mat1->_13*mat2->_34 + mat1->_14*mat2->_44;
    outmat->_21 = mat1->_21*mat2->_11 + mat1->_22*mat2->_21 + mat1->_23*mat2->_31 + mat1->_24*mat2->_41;
    outmat->_22 = mat1->_21*mat2->_12 + mat1->_22*mat2->_22 + mat1->_23*mat2->_32 + mat1->_24*mat2->_42;
    outmat->_23 = mat1->_21*mat2->_13 + mat1->_22*mat2->_23 + mat1->_23*mat2->_33 + mat1->_24*mat2->_43;
    outmat->_24 = mat1->_21*mat2->_14 + mat1->_22*mat2->_24 + mat1->_23*mat2->_34 + mat1->_24*mat2->_44;
    outmat->_31 = mat1->_31*mat2->_11 + mat1->_32*mat2->_21 + mat1->_33*mat2->_31 + mat1->_34*mat2->_41;
    outmat->_32 = mat1->_31*mat2->_12 + mat1->_32*mat2->_22 + mat1->_33*mat2->_32 + mat1->_34*mat2->_42;
    outmat->_33 = mat1->_31*mat2->_13 + mat1->_32*mat2->_23 + mat1->_33*mat2->_33 + mat1->_34*mat2->_43;
    outmat->_34 = mat1->_31*mat2->_14 + mat1->_32*mat2->_24 + mat1->_33*mat2->_34 + mat1->_34*mat2->_44;
    outmat->_41 = mat1->_41*mat2->_11 + mat1->_42*mat2->_21 + mat1->_43*mat2->_31 + mat1->_44*mat2->_41;
    outmat->_42 = mat1->_41*mat2->_12 + mat1->_42*mat2->_22 + mat1->_43*mat2->_32 + mat1->_44*mat2->_42;
    outmat->_43 = mat1->_41*mat2->_13 + mat1->_42*mat2->_23 + mat1->_43*mat2->_33 + mat1->_44*mat2->_43;
    outmat->_44 = mat1->_41*mat2->_14 + mat1->_42*mat2->_24 + mat1->_43*mat2->_34 + mat1->_44*mat2->_44;
    return *outmat;
}
//--------------------------------------------------------------------------------------
MATRIX16 Graphics_IRrad::SetTransformMatrix(MATRIX16 * mat, VECTOR3 * pos)
{//设置平移矩阵,采用行占优的行式,从左到右的顺序相乘
    Mat16SetValue(mat, 1.0f, 0.0f, 0.0f, 0.0f,
                       0.0f, 1.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 1.0f, 0.0f,
                       pos->x, pos->y, pos->z, 1.0f);
    return *mat;
}

//--------------------------------------------------------------------------------------
MATRIX16 Graphics_IRrad::SetRotateMatrix(MATRIX16 * mat, float32 azi, float32 elv, float32 bnk)
{//设置旋转矩阵，根据方位角h(偏航角)绕Y轴、俯仰角p绕X轴、横滚角r绕Z轴进行旋转,传入旋转角单位azi,elv为角度,bnk为弧度

    float32 cosr, cosh, cosp, sinr, sinh, sinp;
    cosr = cos(bnk*pai);
    cosh = cos(azi*pai);
    cosp = cos(elv*pai);
    sinr = sin(bnk*pai);
    sinh = sin(azi*pai);
    sinp = sin(elv*pai);

    /*cosr = cos(bnk);
    cosh = cos(azi);
    cosp = cos(elv);
    sinr = sin(bnk);
    sinh = sin(azi);
    sinp = sin(elv);*/

    Mat16SetValue(mat, cosr*cosh, sinr, -sinh*cosr, 0.0f,
                             -cosp*cosh*sinr+sinp*sinh, cosp*cosr, cosp*sinh*sinr+sinp*cosh, 0.0f,
                             -sinp*cosh*sinr+cosp*sinh, -sinp*cosr, sinp*sinh*sinr+cosp*cosh, 0.0f,
                             0.0f, 0.0f, 0.0f, 1.0f);//xzy
    printf("mat1 is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",mat->_11,mat->_12,mat->_13,mat->_14,mat->_21,mat->_22,mat->_23,mat->_24,mat->_31,mat->_32,mat->_33,mat->_34,mat->_41,mat->_42,mat->_43,mat->_44);


    return *mat;
}

//-------------设置旋转矩阵-----------------------------------------------------
MATRIX16 Graphics_IRrad::SetRotateMatrixQuaternion(MATRIX16 * mat, float32 azi, float32 elv, float32 bnk)
{//设置旋转矩阵，旋转矩阵由traj轨迹中的三个欧拉角得到四元数,然后得到旋转矩阵，和欧拉角有差异，不大。
    float32 q1=0,q2=0, q3=0, q4=0;

    // 由欧拉角创建四元数
    float32 cx = cos(elv*pai/2);
    float32 sx = sin(elv*pai/2);
    float32 cy = cos(azi*pai/2);
    float32 sy = sin(azi*pai/2);
    float32 cz = cos(bnk*pai/2);
    float32 sz = sin(bnk*pai/2);

    q1 = sx*cy*cz - cx*sy*sz;
    q2 = cx*sy*cz + sx*cy*sz;
    q3 = cx*cy*sz - sx*sy*cz;
    q4 = cx*cy*cz + sx*sy*sz;//实部
    //std::cout << "q1" << q1 << "q2" << q2 <<"q3" << q3 <<"q4" << q4 <<std::endl;

    /*float32 sqx = q1 * q1;
    float32 sqy = q2 * q2;
    float32 sqz = q3 * q3;
    bnk = atan2(2.0f * q3 * q4 + 2.0f * q2 * q1, 1.0f - 2.0f * sqz - 2.0f * sqy);
    elv = asin(2.0f * q4 * q2 - 2.0f * q3 * q1);
    azi = atan2(2.0f * q1 * q4 + 2.0f * q3 * q2, 1.0f - 2.0f * sqy - 2.0f * sqx);
    printf("Yaw   = %f\nPitch = %f\nRoll  = %f\n",bnk*180/3.1416,elv*180/3.1416,azi*180/3.1416) ;*/

    float32 qx2 = q1 * q1, qy2 = q2 * q2, qz2 = q3 * q3,
            qxqy = q1 * q2, qxqz = q1 * q3, qyqz = q2 * q3,
            qwqx = q4 * q1, qwqy = q4 * q2, qwqz = q4 * q3;

    /*Mat16SetValue(mat, 1.0f-2.0f*(qy2+qz2),2.0f*(qxqy + qwqz),2.0f*(qxqz-qwqy),0.0f,
                        2.0f*(qxqy - qwqz), 1.0f-2.0f*(qx2+qz2),2.0f*(qyqz+qwqx), 0.0f,
                        2.0f*(qxqz +qwqy), 2.0f*(qyqz-qwqx), 1.0f-2.0f*(qx2+qy2), 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f);*/
    Mat16SetValue(mat, 1.0f-2.0f*(qy2 + qz2),2.0f*(qxqy - qwqz),2.0f*(qxqz + qwqy),0.0f,
                            2.0f*(qxqy + qwqz), 1.0f - 2.0f*(qx2 + qz2),2.0f*(qyqz - qwqx), 0.0f,
                            2.0f*(qxqz - qwqy), 2.0f*(qyqz + qwqx), 1.0f - 2.0f*(qx2 + qy2), 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f);
    //printf("mat1 is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",mat->_11,mat->_12,mat->_13,mat->_14,mat->_21,mat->_22,mat->_23,mat->_24,mat->_31,mat->_32,mat->_33,mat->_34,mat->_41,mat->_42,mat->_43,mat->_44);

    /*azi = atan2(mat->_32,mat->_33);
    elv = atan2(-mat->_31,sqrt(mat->_32 * mat->_32 + mat->_33 * mat->_33));
    bnk = atan2(mat->_21,mat->_11);
    printf("Yaw   = %f\nPitch = %f\nRoll  = %f\n",bnk*180/3.1416,elv*180/3.1416,azi*180/3.1416) ;*/


    return *mat;
}

//--------------------------------------------------------------------------------------
MATRIX16 Graphics_IRrad::SetRotateMatrixVecToVec(MATRIX16 * mat, VECTOR3 * vec1, VECTOR3 * vec2)
{//设置旋转矩阵，由一个单位矢量vec1旋转到另一个单位矢量vec2

    if (vec1->x == vec2->x && vec1->y == vec2->y && vec1->z == vec2->z)
    {
        *mat = Mat16SetIdentity(mat);

        return *mat;
    }
    VECTOR3 vec = Vec3Cross(&vec,vec1, vec2);
    float32 e = Vec3Dot(&e, vec1, vec2);
    float32 vsin = Vec3Dot(&vsin, &vec, &vec) ;
    if (vsin <EPSILON && vsin >-EPSILON)//防止除数为零
    {
        VECTOR3 newvec, newaxis;

        Vec3SetValue(&newvec,vec1->x*1.1f+10.0f,vec1->y*1.2f+20.0f,vec1->z*1.5f+30.0f);//使得newvec与vec1不平行
        Vec3Cross(&newaxis,vec1,&newvec);
        Vec3Normalize(&newaxis,&newaxis);

        //SetRotateMatrixQuaternion(mat,newaxis.x,newaxis.y,newaxis.z,0.0f);

        return *mat;
    }
    float32 h = (1.0f - e) / vsin ;
    Mat16SetValue(mat, e+h*vec.x*vec.x, h*vec.x*vec.y-vec.z, h*vec.x*vec.z+vec.y, 0.0f,
                             h*vec.x*vec.y+vec.z, e+h*vec.y*vec.y, h*vec.y*vec.z-vec.x, 0.0f,
                             h*vec.x*vec.z-vec.y, h*vec.y*vec.z+vec.x, e+h*vec.z*vec.z, 0.0f,
                             0.0f, 0.0f,0.0f, 1.0f);


    return *mat;

}

//------------------设置世界变换矩阵----------------------------------------------------
MATRIX16  Graphics_IRrad::SetWorldMatrix(MATRIX16 * mat, XSTATUS * stat)
{

    MATRIX16 matTrans, matRotate;    
    SetTransformMatrix(&matTrans,&stat->pos);

    //设置世界变换矩阵，旋转信息由traj轨迹中的三个欧拉角得到
    //SetRotateMatrix(&matRotate,stat->azi, stat->elv, stat->bnk);
    //MatrixMultiplyMatrix(mat, &matRotate,&matTrans);
    //printf("mat1 is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",mat->_11,mat->_12,mat->_13,mat->_14,mat->_21,mat->_22,mat->_23,mat->_24,mat->_31,mat->_32,mat->_33,mat->_34,mat->_41,mat->_42,mat->_43,mat->_44);

    //设置世界变换矩阵,旋转矩阵由traj轨迹中的三个欧拉角得到四元组,然后得到旋转矩阵，和欧拉角有差异，不大。
    SetRotateMatrixQuaternion(&matRotate,stat->azi, stat->elv, stat->bnk);
    MatrixMultiplyMatrix(mat, &matRotate,&matTrans);
    //printf("mat2 is (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n",mat->_11,mat->_12,mat->_13,mat->_14,mat->_21,mat->_22,mat->_23,mat->_24,mat->_31,mat->_32,mat->_33,mat->_34,mat->_41,mat->_42,mat->_43,mat->_44);

    return *mat;
}

//--------------------------------------------------------------------------------------
/*MATRIX16  Graphics_IRrad::SetWorldMatrixQRT(MATRIX16 * mat, QRTState * currState)
{//设置世界变换矩阵,旋转矩阵由QRT轨迹中的四元组得到

    MATRIX16 matTrans, matRotate;
    SetTransformMatrix(&matTrans,&currState->pos);
    SetRotateMatrixQuaternion(&matRotate,currState->quaternion.q1, currState->quaternion.q2, currState->quaternion.q3,currState->quaternion.q4);
    MatrixMultiplyMatrix(mat, &matRotate,&matTrans);
    return *mat;//Trans;
}*/

//--------------------------------------------------------------------------------------
MATRIX16  Graphics_IRrad::SetCameraMatrix(MATRIX16 * mat, VECTOR3 * Eye, VECTOR3 * At, VECTOR3 * Up)
{//设置观察变换矩阵
    VECTOR3 xaxis, yaxis, zaxis;
    VECTOR3 AtSubEye, UpCrossZ;
    float32 xdoteye, ydoteye, zdoteye;
    Vec3Subtract(&AtSubEye,At, Eye);//At-Eye
    Vec3Normalize(&zaxis, &AtSubEye);//将AtSubEye归一化->zaxis
    Vec3Cross(&UpCrossZ,Up, &zaxis);//Up和zaxis叉乘
    Vec3Normalize(&xaxis, &UpCrossZ);//UpCrossZ归一化
    Vec3Cross(&yaxis, &zaxis, &xaxis);///old camera matrix, change 20080612 for left-handed coordinates
    /*Vec3Cross(&UpCrossZ, &zaxis,Up);
    Vec3Normalize(&xaxis, &UpCrossZ);
    Vec3Cross(&yaxis, &xaxis,&zaxis );//new camera matrix*/
    //Vec3Normalize(&yaxis, &yaxis);//20160906 zhu add

    Vec3Dot(&xdoteye, &xaxis, Eye);//两个三维矢量点乘
    Vec3Dot(&ydoteye, &yaxis, Eye);
    Vec3Dot(&zdoteye, &zaxis, Eye);
    Mat16SetValue( mat, xaxis.x, yaxis.x, zaxis.x, 0.0f,
                        xaxis.y, yaxis.y, zaxis.y, 0.0f,
                        xaxis.z, yaxis.z, zaxis.z, 0.0f,
                        -xdoteye, -ydoteye, -zdoteye, 1.0f);
    return *mat;
}

//--------------------------------------------------------------------------------------
VECTOR3  Graphics_IRrad::Vec3MultiplyMatrix(VECTOR3 * vec, VECTOR3 * vec1, MATRIX16 * mat, float32 w)
{//vec = vec1 * mat

    //double ww = vec1->x * mat->_14 + vec1->y *mat->_24 + vec1->z * mat->_34 + w * mat->_44;
    //ww = 1.0f/ww;
    vec->x = (vec1->x * mat->_11 + vec1->y * mat->_21 + vec1->z * mat->_31 + w * mat->_41);//*ww;
    vec->y = (vec1->x * mat->_12 + vec1->y * mat->_22 + vec1->z * mat->_32 + w * mat->_42);//*ww;
    vec->z = (vec1->x * mat->_13 + vec1->y * mat->_23 + vec1->z * mat->_33 + w * mat->_43);//*ww;
    return *vec;

}

VECTOR3  Graphics_IRrad::MatrixMultiplyVec3(VECTOR3 * vec,  MATRIX16 * mat, VECTOR3 * vec1,float32 w)
{
    //test only//vec = mat * vec

    //float32 ww = vec1->x * mat->_14 + vec1->y *mat->_24 + vec1->z * mat->_34 + w * mat->_44;
    //ww = 1.0f/ww;
    vec->x = (vec1->x * mat->_11 + vec1->y * mat->_12 + vec1->z * mat->_13 + w * mat->_14);//*ww;
    vec->y = (vec1->x * mat->_21 + vec1->y * mat->_22 + vec1->z * mat->_23 + w * mat->_24);//*ww;
    vec->z = (vec1->x * mat->_31 + vec1->y * mat->_32 + vec1->z * mat->_33 + w * mat->_34);//*ww;
    return *vec;

}
/*
//--------------------------------------------------------------------------------------
VECTOR4 Graphics_IRrad::QuatMultiQuat(VECTOR4 & qprod,VECTOR4 * quat1, VECTOR4 * quat2 )
{// this function multiplies two quaternions

    // this is the brute force method
    //qprod->w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
    //qprod->x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
    //qprod->y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w - q1->z*q2->x;
    //qprod->z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;

    // this method was arrived at basically by trying to factor the above
    // expression to reduce the # of multiplies

    float32 prd_0 = (quat1->q3 - quat1->q2) * (quat2->q2 - quat2->q3);
    float32 prd_1 = (quat1->q4 + quat1->q1) * (quat2->q4 + quat2->q1);
    float32 prd_2 = (quat1->q4 - quat1->q1) * (quat2->q2 + quat2->q3);
    float32 prd_3 = (quat1->q2 + quat1->q3) * (quat2->q4 - quat2->q1);
    float32 prd_4 = (quat1->q3 - quat1->q1) * (quat2->q1 - quat2->q2);
    float32 prd_5 = (quat1->q3 + quat1->q1) * (quat2->q1 + quat2->q2);
    float32 prd_6 = (quat1->q4 + quat1->q2) * (quat2->q4 - quat2->q3);
    float32 prd_7 = (quat1->q4 - quat1->q2) * (quat2->q4 + quat2->q3);

    float32 prd_8 = prd_5 + prd_6 + prd_7;
    float32 prd_9 = 0.5f * (prd_4 + prd_8);

    // and finallq2 build up the result q4ith the temporarq2 products

    qprod.q4 = prd_0 + prd_9 - prd_5;
    qprod.q1 = prd_1 + prd_9 - prd_8;
    qprod.q2 = prd_2 + prd_9 - prd_7;
    qprod.q3 = prd_3 + prd_9 - prd_6;

    return qprod;

} // end QuatMultiQuat

//--------------------------------------------------------------------------------------
VECTOR4 Graphics_IRrad::QUAT_Triple_Product(VECTOR4 & qprod,VECTOR4 *quat1, VECTOR4 *quat2, VECTOR4 *quat3 )
{
    // this function computes q1*q2*q3 in that order and returns
    // the results in qprod

    VECTOR4 qtmp;
    QuatMultiQuat(qtmp,quat1,quat2);
    QuatMultiQuat(qprod,&qtmp, quat3);
    return qprod;
} // end QUAT_Triple_Product*/
//--------------------------------------------------------------------------------------
MATRIX16  Graphics_IRrad::SetPerspectiveMatrix(MATRIX16 * mat, float32 wid, float32 hei, float32 zn, float32 zf )//构建左手坐标系的投影矩阵
{//按此矩阵，xp = (2f/w)*x/z, yp = (2f/h)*y/z, zd = ((z-f)*zfar)/(z*(zfar-f))
    Mat16SetValue( mat, 2.0f * zn / wid, 0.0f, 0.0f, 0.0f,
                        0.0f, 2.0f * zn / hei, 0.0f, 0.0f,
                        0.0f, 0.0f, zf / (zf - zn), 1.0f,
                        0.0f, 0.0f, zn * zf / (zn - zf), 0.0f);
    return *mat;
}

//--------------------------------------------------------------------------------------
MATRIX16  Graphics_IRrad::SetViewportMatrix(MATRIX16 * mat, int32 startX, int32 startY, int32 wid, int32 hei, float32 minZ, float32 maxZ)//设置视区矩阵
{
    Mat16SetValue( mat, 0.5f * wid, 0.0f, 0.0f, 0.0f,
                        0.0f, -0.5f * hei, 0.0f, 0.0f,
                        0.0f, 0.0f, maxZ - minZ,  0.0f,
                        startX + 0.5f * wid, startY + 0.5f * hei, minZ, 1.0f );
    return *mat;
}
//=====================几何一致性数学计算公式==================================


//--------------------------------------------------------------------------------------


//=====================红外辐射一致性辐射温度转换公式==================================
//-----------------计算黑体的辐射强度(W/cm2.sr)-----------------------------------------------
float32 Graphics_IRrad::CalcBlackbodyRadiance(const float32 tmpr)
{
    float32 c1 = 3.7415e4f;//第一辐射常数,单位瓦.厘米^-2.微米^4
    float32 c2 = 1.43879e4f;//第二辐射常数，单位微米.'K
    float32 wl;
    float32 fTempBk = 0.0f, fTempBkSum = 0.0f;
    float32 fBkRadiance = 0.0f;//目标的红外辐射出射度
    for (wl = cam.cpara.camDet.detwl1; wl < cam.cpara.camDet.detwl2; wl += cam.cpara.camDet.detwldelta)
    {
        fTempBk =  1.0f/(pow(wl,5.0f)*(exp(c2/(wl*tmpr))-1.0f));
        fTempBkSum +=  fTempBk ;
    }

    fBkRadiance = fTempBkSum * c1 * cam.cpara.camDet.detwldelta;

    return fBkRadiance;
}

//-------------------计算黑体的红外辐射通量密度的查找表--------------------------------------
float32 * Graphics_IRrad::CalcBbRadianceLookupTable(float32 * rad, float32 IRTmprMin, float32 IRTmprMax, float32 IRTmprDelta)
{
    float32 tmpr = IRTmprMin;
    int32 i = 0;
    int32 num = (int32)((IRTmprMax - IRTmprMin) / IRTmprDelta);
    for (i = 0; i < num; i++)
    {
        rad[i] = CalcBlackbodyRadiance(tmpr);
        tmpr += IRTmprDelta;
    }
    return rad;
}

//-----------------根据指定温度从查找表中找到黑体的红外辐射通量密度-----------------------------------------------
float32 Graphics_IRrad::SearchRadInLookupTable(float32 * radtbl, const float32 tmpr, float32 IRTmprMin, float32 IRTmprMax, float32 IRTmprDelta)
{
    int32 num = (int32)((IRTmprMax - IRTmprMin) / IRTmprDelta);
    int32 i = minimum((int32)( (tmpr - IRTmprMin)/IRTmprDelta ), (num - 1));
    float32 dt1 = (tmpr - i * IRTmprDelta - IRTmprMin) / IRTmprDelta;
    float32 tgtrad = radtbl[i+1] * dt1 + radtbl[i] * (1.0f - dt1);//查找表进行线性插值获得当前目标的黑体红外辐射通量密度(未考虑发射率)
    return tgtrad;
}


//---------------------将红外辐射映射为灰度-----------------------------------------------------
int32 Graphics_IRrad::RadToGray(const float32 rad)//
{
    int32 graytmp;
    float32 Vlsb = (cam.cpara.camDet.advol2-cam.cpara.camDet.advol1)/pow(2.0f,cam.cpara.camDet.adres);
    float32 fCamInceptPower, V;
    fCamInceptPower = ( rad * cam.cpara.camOpt.opttrans * cam.cpara.camDet.uthei * cam.cpara.camDet.utwid *1e-8f* pow(cam.cpara.camOpt.apt,2.0f)) / (4.0f * cam.cpara.camOpt.foc * cam.cpara.camOpt.foc);
    V = fCamInceptPower * (cam.cpara.camDet.detresp/**1e8f*/) * cam.cpara.camDet.adgain + cam.cpara.camDet.adbias;
    graytmp = (int32)(V / Vlsb);
    return graytmp;
}
//=====================红外辐射一致性辐射温度转换公式==================================

//--------------------------------------------------------------------------------------


inline void Mem_Set_QUAD_float32(float32 *dest, float32 data, int32 count)
{//将count个data存到dest指针指向的内存块,data为float32类型

//	_asm//ADSP中不支持这个关键字，具体汇编用asm()，但mov等关键字也不支持。
//	{
//		mov edi, dest;//edi指向目标内存
//		mov ecx, count;//要移动的32位字数
//		mov eax, data;//32位数据
//		rep stosd; //移动数据
//	}
    int32 i;
    for(i=0;i<count;i++)
    {
        dest[i] = data;
    }

}

//--------------------------------------------------------------------------------------
inline void Mem_Set_QUAD(uint32 *dest, uint32 data, int32 count)
{//将count个data存到dest指针指向的内存块,data为uint32类型

//	_asm//ADSP中不支持这个关键字，具体汇编用asm()，但mov等关键字也不支持。
//	{
//		mov edi, dest;//edi指向目标内存
//		mov ecx, count;//要移动的32位字数
//		mov eax, data;//32位数据
//		rep stosd; //移动数据
//	}
    int32 i;
    for(i=0;i<count;i++)
    {
        dest[i] = data;
    }

}




