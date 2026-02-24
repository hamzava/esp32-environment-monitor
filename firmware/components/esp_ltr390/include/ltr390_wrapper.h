#ifndef _ltr390_wrapper_h
#define _ltr390_wrapper_h

void ltr390_cfg_and_init(void);

typedef struct
{
    
    float ambient_light;
    //uint32_t als;
    float ultraviolet_index;
    // uint32_t uvs;

}ltr390_values_t;

void ltr390_data(ltr390_values_t *out);



#endif