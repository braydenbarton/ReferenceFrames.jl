"""
An inertial reference frame (does not accelerate or rotate). Note that this is a *true* inertial frame, which a frame 
like ECI is *not* if the motion of the Earth about the sun is considered

# Properties
- `ID`: The string ID of the frame. Used for display, is not necessarily unique
- `parent`: The reference frame this frame is a child of. If this frame is referenced to no other, then this should be
    `origin`. For example, ECEF is a child of ECI which is a child of HGI which should be `origin` for a problem in
    the solar system
- `q`: The unit quaternion rotating from this frame to its parent
- `r0`: The position of this frame at t=0 in the parent frame
- `v`: The velocity of this frame in the parent frame
"""
struct InertialFrame <: ReferenceFrame
    ID::String
    parent::ReferenceFrame
    q::Vector{<:Real}
    r0::Vector{<:Real}
    v::Vector{<:Real}
end

_framename(::InertialFrame) = "InertialFrame"

getposition(frame::InertialFrame, t::Real) = frame.r0 + frame.v * t
getvelocity(frame::InertialFrame, args...) = frame.v
getacceleration(::InertialFrame, args...) = [0, 0, 0]

getquaternion(frame::InertialFrame, args...) = frame.q
