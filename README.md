# MechanismGeometries

[![Build Status](https://travis-ci.org/JuliaRobotics/MechanismGeometries.jl.svg?branch=master)](https://travis-ci.org/JuliaRobotics/MechanismGeometries.jl) 
[![codecov.io](http://codecov.io/github/JuliaRobotics/MechanismGeometries.jl/coverage.svg?branch=master)](http://codecov.io/github/JuliaRobotics/MechanismGeometries.jl?branch=master)

This package implements several methods of generating or loading geometries associated with a [RigidBodyDynamics.jl](https://github.com/tkoolen/RigidBodyDynamics.jl) `Mechanism` in Julia. It is currently used by [MeshCatMechanisms.jl](https://github.com/JuliaRobotics/MeshCatMechanisms.jl) but can also be used independently. 

# Interface

This package exports one primary method:

```julia
visual_elements(mechanism::Mechanism, source::AbstractGeometrySource)
```

`visual_elements` returns a vector of `VisualElement` structs, each of which contains:

* `frame`: A `CartesianFrame3D` indicating where the geometry is attached in the mechanism
* `geometry`: One of the [GeometryTypes.jl](https://github.com/JuliaGeometry/GeometryTypes.jl) types
* `color`: an RGBA color from [ColorTypes.jl](https://github.com/JuliaGraphics/ColorTypes.jl)
* `transform`: a `Transformation` from [CoordinateTransformations.jl](https://github.com/FugroRoames/CoordinateTransformations.jl/) indicating the pose of the geometry w.r.t its attached frame.

# Currently implemented sources

These demonstrations use the Boston Dynamics Atlas robot from [AtlasRobot.jl](https://github.com/tkoolen/AtlasRobot.jl).

```julia
using AtlasRobot
using MechanismGeometries
mechanism = AtlasRobot.mechanism()
```


## Skeleton

```julia
Skeleton <: AbstractGeometrySource
```

The `Skeleton` type uses only the joints and bodies in the mechanism itself to construct a visual representation of the robot's links. The sticks connect joints in the mechanism and the ellipsoids represent the mass and moment of inertia of each body:

```julia
visual_elements(mechanism, Skeleton())
```

![skeleton_with_inertias](https://user-images.githubusercontent.com/591886/37125808-180c1eca-223c-11e8-8e60-af3f48ba52c2.png)

The moment of inertia ellipsoids can also be turned off, leaving just the joint connections:

```julia
visual_elements(mechanism, Skeleton(inertias=false))
```

![skeleton_no_inertias](https://user-images.githubusercontent.com/591886/37125810-1ac0f4c4-223c-11e8-845a-cf17893eba93.png)

## URDF Visuals

```julia
URDFVisuals <: AbstractGeometrySource
```

The `URDFVisuals` type loads the visual elements from a given URDF file (passed as either a filename or a parsed `XMLDocument` from LightXML.jl). One particularly useful argument is `package_path`, which accepts a list of strings to use as potential directories to search when encountering mesh files using the ROS `package://` syntax.

```julia
visual_elements(mechanism, 
                URDFVisuals(AtlasRobot.urdfpath(), 
                            package_path=[AtlasRobot.packagepath()]))
```

![urdf_visuals](https://user-images.githubusercontent.com/591886/37125819-1de2441e-223c-11e8-9db2-87f814cfea63.png)

### URDF Extensions

The following extensions to the URDF spec are parsed by MechanismGeometries.jl:

* `<plane normal="0 0 1"/>`: Represents an infinite plane perpendicular to the `normal` given as an x y z unit vector. Returns a MechanismGeometries.HyperPlane





