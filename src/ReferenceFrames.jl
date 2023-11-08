module ReferenceFrames
using Quaternion, LinearAlgebra, StaticArrays

export InertialFrame, OriginFrame, transformposition, transformvelocity, transformacceleration, transformquaternion,
    transformdirection

# Declaring convenience type aliases
"Abstract quaternion representation"
const Quat{T} = StaticVector{4, T} where T<:Real
const R3Vec{T} = StaticVector{3, T} where T<:Real

# Include files for frame types
include("ReferenceFrame.jl")
include("InertialFrame.jl")
include("OriginFrame.jl")

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

"Transforms a value between two frames using the given function"
function _transformvalue(
    value, 
    frame1::ReferenceFrame, 
    frame2::ReferenceFrame, 
    t::Real, 
    fn::Function
    )

    # Get the intersection of the two frames' families to determine how to move between them
    frame_inter, family1, family2 = _frame_intersect(frame1, frame2)

    # Ascend the value from frame1 until it reaches frame_inter
    for frame in family1[1:findfirst(x-> x==frame_inter, family1)-1]
        value = fn(value, frame, t, Val(1))
    end

    # Descend the value from frame_inter until it reaches frame2
    for frame in reverse(family2[1:findfirst(x-> x==frame_inter, family2)-1])
        value = fn(value, frame, t, Val(-1))
    end

    return value
end

"Transforms a position vector from a reference frame into its parent frame"
function _shiftposition(
    pos::R3Vec, 
    frame::ReferenceFrame, 
    t::Real,
    ::Val{1}
    )

    f_pos = getposition(frame, t)
    f_quat = getquaternion(frame, t)
    return f_pos + qvq(f_quat, pos)
end

"Transforms a position vector into a reference frame from its parent frame"
function _shiftposition(
    pos::R3Vec, 
    frame::ReferenceFrame, 
    t::Real,
    ::Val{-1}
    )

    f_pos = getposition(frame, t)
    f_quat = qinv(getquaternion(frame, t))
    return qvq(f_quat, pos - f_pos)
end

"Transforms a position vector from frame1 to frame2"
function transformposition(
    pos::R3Vec, 
    frame1::ReferenceFrame, 
    frame2::ReferenceFrame, 
    t::Real
    )

    return _transformvalue(pos, frame1, frame2, t, _shiftposition)
end

"Transforms a velocity vector from a reference frame into its parent frame. Requires position to calculate"
function _shiftvelocity(
    (pos, vel)::Tuple{R3Vec, R3Vec}, 
    frame::ReferenceFrame, 
    t::Real,
    ::Val{1}
    )

    f_vel = getvelocity(frame, t)
    f_quat = getquaternion(frame, t)
    f_omega = getomega(frame, t)
    pos_new = _shiftposition(pos, frame, t, Val(1))
    vel_new = f_vel + qvq(f_quat, vel) + cross(f_omega, qvq(f_quat, pos))
    return pos_new, vel_new
end

"Transforms a velocity vector into a reference frame from its parent frame. Requires position to calculate"
function _shiftvelocity(
    (pos, vel)::Tuple{R3Vec, R3Vec}, 
    frame::ReferenceFrame, 
    t::Real,
    ::Val{-1}
    )

    f_vel = getvelocity(frame, t)
    f_quat = qinv(getquaternion(frame, t))
    f_omega = getomega(frame, t)
    pos_new = _shiftposition(pos, frame, t, Val(-1))
    vel_new = qvq(f_quat, vel - f_vel - cross(f_omega, pos))
    return pos_new, vel_new
end

"Transforms a velocity vector from frame1 to frame2"
function transformvelocity(
    (pos, vel)::Tuple{R3Vec, R3Vec}, 
    frame1::ReferenceFrame, 
    frame2::ReferenceFrame, 
    t::Real
    )

    return _transformvalue((pos, vel), frame1, frame2, t, _shiftvelocity)
end
transformvelocity(pos::R3Vec, vel::R3Vec, frame1::ReferenceFrame, frame2::ReferenceFrame, t::Real) = 
    transformvelocity((pos, vel), frame1, frame2, t)

"Transforms an acceleration vector from a reference frame into its parent frame. Requires position and velocity"
function _shiftaccel(
    (pos, vel, acc)::Tuple{R3Vec, R3Vec, R3Vec}, 
    frame::ReferenceFrame, 
    t::Real,
    ::Val{1}
    )

    f_acc = getacceleration(frame, t)
    f_quat = getquaternion(frame, t)
    f_omega = getomega(frame, t)
    f_alpha = getalpha(frame, t)
    pos_r = qvq(f_quat, pos)
    vel_r = qvq(f_quat, vel)
    acc_r = qvq(f_quat, acc)
    pos_new, vel_new = _shiftvelocity((pos, vel), frame, t, Val(1))
    acc_new = acc_r + f_acc + 2*cross(f_omega, vel_r) + cross(f_omega, cross(f_omega, pos_r)) + cross(f_alpha, pos_r)
    return pos_new, vel_new, acc_new
end

"Transforms an acceleration vector into a reference frame from its parent frame. Requires position and velocity"
function _shiftaccel(
    (pos, vel, acc)::Tuple{R3Vec, R3Vec, R3Vec}, 
    frame::ReferenceFrame, 
    t::Real,
    ::Val{-1}
    )

    f_acc = getacceleration(frame, t)
    f_quat = getquaternion(frame, t)
    f_omega = getomega(frame, t)
    f_alpha = getalpha(frame, t)
    pos_new, vel_new = _shiftvelocity((pos, vel), frame, t, Val(-1))
    acc_new = acc - f_acc - 2*cross(f_omega, vel) - cross(f_omega, cross(f_omega, pos)) - cross(f_alpha, pos)
    acc_new = qvq(f_quat, acc_new)
    return pos_new, vel_new, acc_new
end

"Transforms an acceleration vector from frame1 to frame2"
function transformacceleration(
    (pos, vel, acc)::Tuple{R3Vec, R3Vec, R3Vec},
    frame1::ReferenceFrame, 
    frame2::ReferenceFrame, 
    t::Real
    )

    return _transformvalue((pos, vel, acc), frame1, frame2, t, _shiftaccel)
end
transformacceleration(pos::R3Vec, vel::R3Vec, acc::R3Vec, frame1::ReferenceFrame, frame2::ReferenceFrame, t::Real) =
    transformacceleration((pos, vel, acc), frame1, frame2, t)

"Rotate a quaternion from a reference frame to its parent's axes"
function _shiftquat(
    quat::Quat,
    frame::ReferenceFrame,
    t::Real,
    ::Val{1}
    )

    f_quat = getquaternion(frame, t)
    return f_quat ⊗ quat
end

"Rotate a quaternion into a reference frame from its parent's axes"
function _shiftquat(
    quat::Quat,
    frame::ReferenceFrame,
    t::Real,
    ::Val{-1}
    )

    f_quat = qconj(getquaternion(frame, t))
    return f_quat ⊗ quat
end

"Rotate a quaternion from frame1 to frame2"
function transformquaternion(
    quat::Quat,
    frame1::ReferenceFrame,
    frame2::ReferenceFrame,
    t::Real
    )

    return _transformvalue(quat, frame1, frame2, t, _shiftquat)
end

"Transform a direction vector from frame1 to frame2"
function transformdirection(
    vec::R3Vec,
    frame1::ReferenceFrame,
    frame2::ReferenceFrame,
    t::Real
    )

    return transformquaternion([0; vec], frame1, frame2, t)[SA[2:4...]]
end

end
