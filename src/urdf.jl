module URDF

using LightXML
using RigidBodyDynamics
using RigidBodyDynamics.Graphs
const rbd = RigidBodyDynamics
using ColorTypes: RGBA
using GeometryTypes
using MechanismGeometries: VisualElement, DEFAULT_COLOR, AbstractGeometrySource, HyperPlane
import MechanismGeometries: visual_elements
using CoordinateTransformations: AffineMap
using MeshIO
using FileIO: load

export URDFVisuals

function parse_geometries(xml_geometry::XMLElement, package_path, file_path="")
    geometries = Union{AbstractGeometry, AbstractMesh}[]
    for xml_cylinder in get_elements_by_tagname(xml_geometry, "cylinder")
        length = rbd.parse_scalar(Float32, xml_cylinder, "length")
        radius = rbd.parse_scalar(Float32, xml_cylinder, "radius")
        extent = Point(0, 0, length)
        push!(geometries, Cylinder{3, Float32}(Point(0, 0, -length/2),
                                               Point(0, 0, length/2), radius))
    end
    for xml_box in get_elements_by_tagname(xml_geometry, "box")
        size = Vec{3, Float32}(rbd.parse_vector(Float32, xml_box, "size", "0 0 0"))
        push!(geometries, HyperRectangle(-size / 2, size))
    end
    for xml_sphere in get_elements_by_tagname(xml_geometry, "sphere")
        radius = rbd.parse_scalar(Float32, xml_sphere, "radius")
        push!(geometries, HyperSphere(zero(Point{3, Float32}), radius))
    end
    for xml_plane in get_elements_by_tagname(xml_geometry, "plane")
        normal = Vec{3, Float32}(rbd.parse_vector(Float32, xml_plane, "normal", "0 0 1"))
        push!(geometries, HyperPlane(normal))
    end
    for xml_mesh in get_elements_by_tagname(xml_geometry, "mesh")
        filename = attribute(xml_mesh, "filename")
        dae_pattern = r".dae$"
        replaced_extension_with_obj = false
        if occursin(dae_pattern, filename)
            filename = replace(filename, dae_pattern => ".obj")
            replaced_extension_with_obj = true
        end
        package_pattern = r"^package://"

        if occursin(package_pattern, filename)
            found_mesh = false
            for package_directory in package_path
                filename_in_package = joinpath(package_directory, replace(filename, package_pattern => ""))
                if ispath(filename_in_package)
                    mesh = load(filename_in_package, GLUVMesh)
                    push!(geometries, mesh)
                    found_mesh = true
                    break
                end
            end
            if !found_mesh
                warning_message = "Could not find the mesh file: $(filename). I tried substituting the following folders for the 'package://' prefix: $(package_path)."
                if replaced_extension_with_obj
                    warning_message *= " Note that I replaced the file's original extension with .obj to try to find a mesh in a format I can actually load."
                end
                @warn(warning_message)
            end
        else
            filename = joinpath(file_path, filename)
            if ispath(filename)
                mesh = load(filename)
                push!(geometries, mesh)
            else
                warning_message = "Could not find the mesh file: $(filename)."
                if replaced_extension_with_obj
                    warning_message *= " Note that I replaced the file's original extension with .obj to try to find a mesh in a format I can actually load."
                end
                @warn(warning_message)
            end
        end
    end
    geometries
end

function parse_material!(material_colors::Dict{String, RGBA{Float32}}, xml_material)
    if xml_material === nothing
        return DEFAULT_COLOR
    end
    name = attribute(xml_material, "name")
    xml_color = find_element(xml_material, "color")
    if xml_color !== nothing
        default = "0.7 0.7 0.7 1."
        material_colors[name] = RGBA{Float32}(rbd.parse_vector(Float32, xml_color, "rgba", default)...)
    end
    get(material_colors, name, DEFAULT_COLOR)
end

function parse_link!(material_colors::Dict, xml_link,
                     package_path=ros_package_path(), file_path="", tag="visual", link_colors=Dict{String, RGBA{Float32}}())
    ret = []
    xml_visuals = get_elements_by_tagname(xml_link, tag)
    for xml_visual in xml_visuals
        xml_material = find_element(xml_visual, tag)
        linkname = attribute(xml_link, "name")
        color = get(link_colors, linkname, parse_material!(material_colors, find_element(xml_visual, "material")))
        rot, trans = rbd.parse_pose(Float64, find_element(xml_visual, "origin"))
        tform = AffineMap(rot, trans)
        for geometry in parse_geometries(find_element(xml_visual, "geometry"), package_path, file_path)
            push!(ret, (geometry, color, tform))
        end
    end
    ret
end

function create_graph(xml_links, xml_joints)
    # create graph structure of XML elements
    graph = DirectedGraph{Vertex{XMLElement}, Edge{XMLElement}}()
    vertices = Vertex.(xml_links)
    for vertex in vertices
        add_vertex!(graph, vertex)
    end
    name_to_vertex = Dict(attribute(v.data, "name") => v for v in vertices)
    for xml_joint in xml_joints
        parent = name_to_vertex[attribute(find_element(xml_joint, "parent"), "link")]
        child = name_to_vertex[attribute(find_element(xml_joint, "child"), "link")]
        add_edge!(graph, parent, child, Edge(xml_joint))
    end
    graph
end

ros_package_path() = split(get(ENV, "ROS_PACKAGE_PATH", ""), ':')

struct URDFVisuals <: AbstractGeometrySource
    xdoc::XMLDocument
    package_path::Vector{String}
    file_path::String
    tag::String
    link_colors::Dict{String, RGBA{Float32}}
end

function URDFVisuals(xdoc::XMLDocument; package_path=ros_package_path(), file_path="", tag="visual", link_colors=Dict{String, RGBA{Float32}}())
    URDFVisuals(xdoc, package_path, file_path, tag, link_colors)
end

URDFVisuals(filename::AbstractString; kw...) = URDFVisuals(parse_file(filename); kw...)

function visual_elements(mechanism::Mechanism, source::URDFVisuals)
    xroot = LightXML.root(source.xdoc)
    @assert LightXML.name(xroot) == "robot"

    xml_links = get_elements_by_tagname(xroot, "link")
    xml_joints = get_elements_by_tagname(xroot, "joint")
    xml_materials = get_elements_by_tagname(xroot, "material")

    graph = create_graph(xml_links, xml_joints)
    roots = collect(filter(v -> isempty(in_edges(v, graph)), rbd.Graphs.vertices(graph)))
    length(roots) != 1 && error("Can only handle a single root")
    tree = SpanningTree(graph, first(roots))

    material_colors = Dict{String, RGBA{Float32}}()
    for xml_material in xml_materials
        parse_material!(material_colors, xml_material)
    end

    name_to_frame = Dict(string(tf.from) => tf.from for body in bodies(mechanism) for tf in rbd.frame_definitions(body))

    elements = Vector{VisualElement}()
    for vertex in rbd.Graphs.vertices(tree)
        xml_link = vertex.data

        linkname = attribute(xml_link, "name")
        framename = if vertex == rbd.Graphs.root(tree)
            linkname
        else
            xml_joint = edge_to_parent(vertex, tree).data
            jointname = attribute(xml_joint, "name")
            string("after_", jointname) # TODO: create function in RBD, call it here
        end
        if haskey(name_to_frame, framename)
            body_frame = name_to_frame[framename]
            for (geometry, color, tform) in parse_link!(material_colors, xml_link, source.package_path, source.file_path, source.tag, source.link_colors)
                push!(elements, VisualElement(body_frame, geometry, color, tform))
            end
        end
    end
    elements
end

end