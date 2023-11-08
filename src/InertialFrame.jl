"""
A reference frame that does not accelerate or rotate relative to its parent. Note that this is only a *true* inertial frame if
all frames above it in the hierarchy are a subtype of `AbstractInertialFrame`

# Properties
- `ID`: The string ID of the frame. Used for display, is not necessarily unique
- `parent`: The reference frame this frame is a child of. For example, ECEF is a child of ECI which is a child of HGI 
    which should be an `OriginFrame` for a problem in the solar system
- `q`: The unit quaternion rotating from this frame to its parent
- `r0`: The position of this frame at t=0 in the parent frame
- `v`: The velocity of this frame in the parent frame
"""
struct InertialFrame <: AbstractInertialFrame
    ID::String
    parent::ReferenceFrame
    q::Quat
    r0::R3Vec
    v::R3Vec

    "Inner constructor taking keyword arguments only, allowing default values"
    function InertialFrame(;
        ID::Union{String, Nothing}=nothing,
        parent::ReferenceFrame,
        q::Quat=SA[1, 0, 0, 0],
        r0::R3Vec=SA[0, 0, 0],
        v::R3Vec=SA[0, 0 ,0]
        )

        # If no ID supplied, create it automatically from a hash of the inputs
        if isnothing(ID)
            ID = _hashid(8, parent, q, r0, v)
        end

        new(ID, parent, q, r0, v)
    end
end

# Constructors for taking positional arguments
InertialFrame(ID::String, parent::ReferenceFrame, q::Quat, r0::R3Vec, v::R3Vec) = 
    InertialFrame(ID=ID, parent=parent, q=q, r0=r0, v=v)

InertialFrame(parent::ReferenceFrame, q::Quat, r0::R3Vec, v::R3Vec) = InertialFrame(parent=parent, q=q, r0=r0, v=v)
InertialFrame(parent::ReferenceFrame, q::Quat) = InertialFrame(parent=parent, q=q)

getposition(frame::InertialFrame, t::Real) = frame.r0 + frame.v * t
getvelocity(frame::InertialFrame, args...) = frame.v
getquaternion(frame::InertialFrame, args...) = frame.q
