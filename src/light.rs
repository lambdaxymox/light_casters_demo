use cglinalg::{
    Vector3,
    ScalarFloat,
};

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PointLight<S> {
    pub ambient: Vector3<S>,
    pub diffuse: Vector3<S>,
    pub specular: Vector3<S>,
}

impl<S> PointLight<S> where S: ScalarFloat {
    pub fn new(ambient: Vector3<S>, diffuse: Vector3<S>, specular: Vector3<S>) -> PointLight<S> {
        PointLight {
            ambient: ambient,
            diffuse: diffuse,
            specular: specular,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SpotLight<S> {
    pub position: Vector3<S>,  
    pub direction: Vector3<S>,
    pub cutoff: S,
    pub outer_cutoff: S,
      
    pub ambient: Vector3<S>,
    pub diffuse: Vector3<S>,
    pub specular: Vector3<S>,

    pub constant: S,
    pub linear: S,
    pub quadratic: S,
}

impl<S> SpotLight<S> where S: ScalarFloat {
    pub fn new(
        position: Vector3<S>,  
        direction: Vector3<S>,
        cutoff: S,
        outer_cutoff: S,
        ambient: Vector3<S>,
        diffuse: Vector3<S>,
        specular: Vector3<S>,
        constant: S,
        linear: S,
        quadratic: S) -> SpotLight<S> 
    {
        SpotLight {
            position: position,  
            direction: direction,
            cutoff: cutoff,
            outer_cutoff: outer_cutoff,
            ambient: ambient,
            diffuse: diffuse,
            specular: specular,
            constant: constant,
            linear: linear,
            quadratic: quadratic,
        }
    }   
}

