#ifndef __3PH_H__
#define __3PH_H__

// Structure Definition DQ0 -> abc
typedef struct{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
    float sin;
    float cos;
    float d;
    float q;
    float z;
} DQ0_ABC;

// Structure Definition ABC->DQ0
typedef struct{
    float a;
    float b;
    float c;
    float alpha;
    float beta;
    float sin;
    float cos;
    float d;
    float q;
    float z;
} ABC_DQ0_POS;

// Structure Definition PLL
typedef struct{
    float   B1_lf;
    float   B0_lf;
    float   A1_lf;
} SPLL_3ph_SRF_LPF_COEFF;

typedef struct{
    float v_q[2];
    float ylf[2];
    float fo; // output frequency of PLL
    float fn; //nominal frequency
    float theta[2];
    float delta_T;
    SPLL_3ph_SRF_LPF_COEFF lpf_coeff;
} SPLL_3ph_SRF;


void DQ0_ABC_init(DQ0_ABC *v);
void DQ0_ABC_FUNC(DQ0_ABC *v);
void ABC_DQ0_POS_init(ABC_DQ0_POS *v);
void ABC_DQ0_POS_FUNC(ABC_DQ0_POS *v);
void SPLL_3ph_SRF_init(int Grid_freq, float DELTA_T, SPLL_3ph_SRF *spll_obj);
void SPLL_3ph_SRF_FUNC(SPLL_3ph_SRF *spll_obj);


// Macros (for implementation in CLA)
#define DQ0_ABC_INIT_MACRO(v)\
    v.a=0;\
    v.b=0;\
    v.c=0;\
    v.alpha=0;\
    v.beta=0;\
    v.z=0;\
    v.d=0;\
    v.q=0;\

#define DQ0_ABC_FUNC_MACRO(v)\
        v.alpha = v.d*v.cos - v.q*v.sin;\
        v.beta  = v.d*v.sin + v.q*v.cos;\
        v.a = v.alpha + 0.5*v.z;\
        v.b = -0.5*v.alpha + 0.8660254*v.beta + 0.5*v.z;\
        v.c = -0.5*v.alpha - 0.8660254*v.beta + 0.5*v.z;\

#define ABC_DQ0_POS_INIT_MACRO(v)\
        v.a=0;\
        v.b=0;\
        v.c=0;\
        v.alpha=0;\
        v.beta=0;\
        v.z=0;\
        v.d=0;\
        v.q=0;\

#define ABC_DQ0_POS_FUNC_MACRO(v)\
    v.alpha=(0.6666666667)*(v.a-0.5*(v.b+v.c));\
    v.beta=(0.57735026913)*(v.b-v.c);\
    v.z =0.57735026913*(v.a+v.b+v.c);\
    v.d=v.alpha*v.cos+v.beta*v.sin;\
    v.q=-v.alpha*v.sin+v.beta*v.cos;\

#define SPLL_3ph_SRF_INIT_MACRO(Grid_freq, DELTA_T, v)\
        v.v_q[0]=(0.0);\
        v.v_q[1]=(0.0);\
        v.ylf[0]=(0.0);\
        v.ylf[1]=(0.0);\
        v.fo=(0.0);\
        v.fn=(Grid_freq);\
        v.theta[0]=(0.0);\
        v.theta[1]=(0.0);\
        v.lpf_coeff.B0_lf=(222.685);\
        v.lpf_coeff.B1_lf=(-221.636);\
        v.lpf_coeff.A1_lf=(-1.0);\
        v.delta_T=DELTA_T;\

#define SPLL_3ph_SRF_FUNC_MACRO(v)\
    v.ylf[0]=v.ylf[1] + (v.lpf_coeff.B0_lf*v.v_q[0]) + (v.lpf_coeff.B1_lf*v.v_q[1]);\
    v.ylf[1]=v.ylf[0];\
    v.v_q[1]=v.v_q[0];\
    v.ylf[0]=(v.ylf[0]>(200.0))?(200.0):v.ylf[0];\
    v.fo=v.fn + v.ylf[0];\
    v.theta[0]=v.theta[1] + ((v.fo*v.delta_T)*(2.0*3.1415926));\
    if(v.theta[0] > (2.0*3.1415926))\
        v.theta[0]=v.theta[0] - (2.0*3.1415926);\
    v.theta[1]=v.theta[0];\

#endif // __PLL_H__
