module ReferenceFrames
using Quaternion

export show, origin, OriginFrame, InertialFrame, transformposition

"Supertype for all reference frame types"
abstract type ReferenceFrame end

"""
    getfamily(frame::ReferenceFrame)

Returns a `Vector` of `ReferenceFrame` objects corresponding with the frame's parents, those parents' parents, and so 
on, until an `OriginFrame` is reached.

For example, if called on frame A, and frame A is a child of frame B, frame B is a child of frame C, and frame C is an
`OriginFrame`, it will return 
`[A, B, C]`
"""
getfamily(frame::ReferenceFrame) = [frame; getfamily(frame.parent)]

"Display method for ReferenceFrames"
Base.show(io::IO, frame::ReferenceFrame) = print(io, "$(_framename(frame))($(frame.ID))")

"Transforms a position vector from a reference frame into its parent frame"
function ascendposition(frame::ReferenceFrame, pos::Vector{<:Real}, t::Real)
    f_pos = getposition(frame, t)
    f_quat = getquaternion(frame, t)
    return f_pos + qvq(f_quat, pos)
end

"Transforms a position vector into a reference frame from its parent frame"
function descendposition(frame::ReferenceFrame, pos::Vector{<:Real}, t::Real)
    f_pos = getposition(frame, t)
    f_quat = getquaternion(frame, t)
    return qvq(qinv(f_quat), pos - f_pos)
end

"Finds the intersection of two frames' families"
function _frame_intersect(frame1::ReferenceFrame, frame2::ReferenceFrame)
    family1 = getfamily(frame1)
    family2 = getfamily(frame2)
    family_intersect = intersect(family1, family2)
    if isempty(family_intersect)
        error("Frames $frame1 and $frame2 do not have a common hierarchy")
    end

    return family_intersect[1], family1, family2
end

"Transforms a position vector from frame1 to frame2"
function transformposition(frame1::ReferenceFrame, frame2::ReferenceFrame, pos::Vector{<:Real}, t::Real)
    # Get the intersection of the two frames' families to determine how to move between them
    frame_inter, family1, family2 = _frame_intersect(frame1, frame2)

    # Ascend the position vector from frame1 until it reaches frame_inter
    for frame in family1[1:findfirst(x-> x==frame_inter, family1)-1]
        pos = ascendposition(frame, pos, t)
    end

    # Descend the position vector from frame_inter until it reaches frame2
    for frame in reverse(family2[1:findfirst(x-> x==frame_inter, family2)-1])
        pos = descendposition(frame, pos, t)
    end
    
    return pos
end

"""
All reference frames will descend from an OriginFrame in some way. This type has no data besides an ID, but rather
serves as the final point in the frame hierarchy. The `origin` object declared below the OriginFrame definition is the 
default origin. If multiple unrelated frame hierarchies are required, a new origin must be declared for each

# Properties
- `ID`: The string ID of the frame. Used for display, is not necessarily unique
"""
struct OriginFrame <: ReferenceFrame 
    ID::String
end
const origin = OriginFrame("Default Origin")

_framename(::OriginFrame) = "OriginFrame"

getposition(::OriginFrame, args...) = [0, 0, 0]
getvelocity(::OriginFrame, args...) = [0, 0, 0]
getacceleration(::OriginFrame, args...) = [0, 0, 0]

getquaternion(::OriginFrame, args...) = [1, 0, 0, 0]

"When called on an `OriginFrame` object, return the object itself"
getfamily(frame::OriginFrame) = [frame]


include("InertialFrame.jl")

end
