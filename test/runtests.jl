using MechanismGeometries
using RigidBodyDynamics
const rbd = RigidBodyDynamics
using GeometryTypes
using StaticArrays: SVector, SDiagonal
using CoordinateTransformations: AffineMap, IdentityTransformation, LinearMap, Translation, transform_deriv
using ColorTypes: RGBA, BGR
using ValkyrieRobot
using Compat
using Compat.Test
using Compat.LinearAlgebra

function homog(::IdentityTransformation)
    Matrix(1.0I, 4, 4)
end

function homog(m::AffineMap)
    H = Matrix(1.0I, 4, 4)
    H[1:3, 1:3] .= transform_deriv(m, SVector(0., 0., 0.))
    H[1:3, 4] .= m(SVector(0., 0., 0.))
    H
end

homog(m::LinearMap) = homog(m ∘ Translation(0, 0, 0))

homog(t::Translation) = homog(AffineMap(Matrix(1.0I, 3, 3), t(SVector(0., 0., 0.))))

@testset "MechanismGeometries" begin
    @testset "skeletons" begin
        @testset "unit sphere" begin
            frame = CartesianFrame3D("com")
            inertia = SpatialInertia(frame, SDiagonal(1., 1, 1), SVector(0., 0, 0), 1.0)
            # I = diag([1, 1, 1]), M = 1
            # so the equivalent ellipsoid must have the same moment of inertia
            # Ixx = 1/5 * M * (b^2 + c^2)
            # so 1 = 1/5 * (b^2 + c^2) and b = c
            # so 1 = 2/5 b^2
            # and b^2 = 5/2 -> b = sqrt(5/2)
            element = MechanismGeometries.inertial_ellipsoid(inertia)
            @test element.frame === frame
            @test homog(element.transform) ≈ homog(AffineMap(SDiagonal(sqrt(5/2), sqrt(5/2), sqrt(5/2)), SVector(0., 0, 0)))
            @test element.geometry isa HyperSphere
            @test radius(element.geometry) ≈ 1
        end

        @testset "heavy sphere" begin
            frame = CartesianFrame3D("com")
            inertia = SpatialInertia(frame, SDiagonal(1., 1, 1), SVector(0., 0, 0), 5.0)
            # I = diag([1, 1, 1]), M = 5
            # so the equivalent ellipsoid must have the same moment of inertia
            # Ixx = 1/5 * M * (b^2 + c^2)
            # so 1 = 1/5 * 5 * (b^2 + c^2) and b = c
            # so 1 = 2 b^2
            # and b^2 = 1/2 -> b = sqrt(1/2)
            element = MechanismGeometries.inertial_ellipsoid(inertia)
            @test element.frame === frame
            @test homog(element.transform) ≈ homog(LinearMap(SDiagonal(sqrt(1/2), sqrt(1/2), sqrt(1/2))))
            @test element.geometry isa HyperSphere
            @test radius(element.geometry) ≈ 1
        end

        @testset "translated sphere" begin
            frame = CartesianFrame3D("com")
            inertia = SpatialInertia(frame, SDiagonal(1., 1, 1), SVector(1., 0, 0), 5.0)
            @test center_of_mass(inertia).v ≈ [0.2, 0, 0]
            element = MechanismGeometries.inertial_ellipsoid(inertia)
            @test element.frame === frame
            @test element.transform(SVector(0, 0, 0)) ≈ center_of_mass(inertia).v
            @test element.geometry isa HyperSphere
            @test radius(element.geometry) ≈ 1
        end
    end

    @testset "urdf" begin
        @testset "acrobot" begin
            urdf = "urdf/Acrobot.urdf"
            robot = parse_urdf(Float64, urdf)
            elements = visual_elements(robot, URDFVisuals(urdf))
            @test length(elements) == 3

            element = elements[1]
            @test string(rbd.body_fixed_frame_to_body(robot, element.frame)) == "base_link"
            @test element.geometry isa HyperRectangle
            @test element.geometry.origin ≈ [-0.1, -0.1, -0.1]
            @test element.geometry.widths ≈ [0.2, 0.2, 0.2]
            @test element.color == RGBA(0, 1, 0, 1)
            @test homog(element.transform) ≈ homog(IdentityTransformation())

            element = elements[2]
            @test string(rbd.body_fixed_frame_to_body(robot, element.frame)) == "upper_link"
            @test element.geometry isa Cylinder
            @test element.geometry.origin ≈ [0, 0, -1.1 / 2]
            @test element.geometry.extremity ≈ [0, 0, 1.1 / 2]
            @test radius(element.geometry) ≈ 0.05
            @test element.color == RGBA(1, 0, 0, 1)
            @test homog(element.transform) ≈ homog(Translation(0, 0, -0.5))

            element = elements[3]
            @test string(rbd.body_fixed_frame_to_body(robot, element.frame)) == "lower_link"
            @test element.geometry isa Cylinder
            @test element.geometry.origin ≈ [0, 0, -2.1 / 2]
            @test element.geometry.extremity ≈ [0, 0, 2.1 / 2]
            @test radius(element.geometry) ≈ 0.05
            @test element.color == RGBA(0, 0, 1, 1)
            @test homog(element.transform) ≈ homog(Translation(0, 0, -1))
        end

        @testset "acrobot with fixed elbow" begin
            urdf = "urdf/Acrobot_fixed.urdf"
            robot = parse_urdf(Float64, urdf)
            rbd.remove_fixed_tree_joints!(robot)
            elements = visual_elements(robot, URDFVisuals(urdf))
            @test length(elements) == 3

            element = elements[1]
            @test string(rbd.body_fixed_frame_to_body(robot, element.frame)) == "world"
            @test element.geometry isa HyperRectangle
            @test element.geometry.origin ≈ [-0.1, -0.1, -0.1]
            @test element.geometry.widths ≈ [0.2, 0.2, 0.2]
            @test element.color == RGBA(0, 1, 0, 1)
            @test homog(element.transform) ≈ homog(IdentityTransformation())

            element = elements[2]
            @test string(rbd.body_fixed_frame_to_body(robot, element.frame)) == "upper_link"
            @test element.geometry isa Cylinder
            @test element.geometry.origin ≈ [0, 0, -1.1 / 2]
            @test element.geometry.extremity ≈ [0, 0, 1.1 / 2]
            @test radius(element.geometry) ≈ 0.05
            @test element.color == RGBA(1, 0, 0, 1)
            @test homog(element.transform) ≈ homog(Translation(0, 0, -0.5))

            element = elements[3]
            @test string(rbd.body_fixed_frame_to_body(robot, element.frame)) == "upper_link"
            @test element.geometry isa Cylinder
            @test element.geometry.origin ≈ [0, 0, -2.1 / 2]
            @test element.geometry.extremity ≈ [0, 0, 2.1 / 2]
            @test radius(element.geometry) ≈ 0.05
            @test element.color == RGBA(0, 0, 1, 1)
            @test homog(element.transform) ≈ homog(Translation(0, 0, -1))
        end

        @testset "acrobot submechanism" begin
            urdf = "urdf/Acrobot.urdf"
            robot = parse_urdf(Float64, urdf)
            s, _ = submechanism(robot, bodies(robot)[3])
            elements = visual_elements(s, URDFVisuals(urdf))
            @test length(elements) == 2
            @test string(rbd.body_fixed_frame_to_body(robot, elements[1].frame)) == "upper_link"
            @test string(rbd.body_fixed_frame_to_body(robot, elements[2].frame)) == "lower_link"
        end

        @testset "ground plane" begin
            urdf = "urdf/ground_plane.urdf"
            robot = parse_urdf(Float64, urdf)
            elements = visual_elements(robot, URDFVisuals(urdf; tag="collision"))
            @test length(elements) == 2
            @test elements[1].frame === elements[2].frame
            @test elements[1].geometry isa HyperRectangle
            @test elements[2].geometry isa HyperPlane
            @test elements[2].geometry.normal == Vec(0., 0, 1)
            @test homog(elements[2].transform) ≈ homog(Translation(0, 0, 0.025))
        end

        @testset "file with missing mesh" begin
            urdf = "urdf/missing_meshfile.urdf"
            robot = parse_urdf(Float64, urdf)
            elements = visual_elements(robot, URDFVisuals(urdf; tag="collision"))
        end

        @testset "link_colors keyword argument" begin
            urdf = "urdf/Acrobot.urdf"
            robot = parse_urdf(Float64, urdf)
            link = last(bodies(robot))
            for override_color in [RGBA(0.1f0, 0.2f0, 0.3f0, 0.4f0), BGR(0.1, 0.2, 0.3)]
                link_colors = Dict(string(link) => override_color)
                elements_override = visual_elements(robot, URDFVisuals(urdf; link_colors=link_colors))
                elements_base = visual_elements(robot, URDFVisuals(urdf))
                for (element_override, element_base) in zip(elements_override, elements_base)
                    if element_base == last(elements_base)
                        @test RGBA{Float32}(element_override.color) == RGBA{Float32}(override_color)
                    else
                        @test element_override.color == element_base.color
                    end
                end
            end
        end
    end

    @testset "valkyrie" begin
        robot = ValkyrieRobot.Valkyrie().mechanism
        visual_elements(robot, Skeleton())
        visual_elements(robot, Skeleton(inertias=false, randomize_colors=true))
        visual_elements(robot, URDFVisuals(ValkyrieRobot.urdfpath(), package_path=[ValkyrieRobot.packagepath()]))
    end

end
