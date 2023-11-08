"Abstract supertype for all reference frames"
abstract type ReferenceFrame end

"Abstract supertype for inertial frames"
abstract type AbstractInertialFrame <: ReferenceFrame end

# Standard methods for inertial frames
getacceleration(::AbstractInertialFrame, args...) = SA[0, 0, 0]
getomega(::AbstractInertialFrame, args...) = @SVector zeros(3)
getalpha(::AbstractInertialFrame, args...) = @SVector zeros(3)

"""
    getfamily(frame::ReferenceFrame)

Returns a `Vector` of `ReferenceFrame` objects corresponding with the frame's parents, those parents' parents, and so 
on, until an `OriginFrame` is reached.

For example, if called on frame A, and frame A is a child of frame B, frame B is a child of frame C, and frame C is an
`OriginFrame`, it will return 
`[A, B, C]`
"""
getfamily(frame::ReferenceFrame) = [frame; getfamily(getparent(frame))]

"Default parent fetching method for ReferenceFrame objects"
getparent(frame::ReferenceFrame) = frame.parent

"Check if a reference frame is a true inertial reference frame"
isinertial(frame::ReferenceFrame) = frame isa AbstractInertialFrame && getparent(frame) isa AbstractInertialFrame

"Display method for ReferenceFrames"
Base.show(io::IO, frame::ReferenceFrame) = print(io, "$(_framename(frame))($(frame.ID))")

"Default method for getting a frame's type name"
_framename(frame::ReferenceFrame) = string(typeof(frame))

"Creates a mostly-unique ID with the specified number of digits from the hash of some input values. The more digits in 
the ID, the more likely it is to give unique IDs"
function _hashid(ndigs::Int64, args...)
    hashdigs = digits(hash(args, hash(args)), base=2, pad=64)  # The binary digits of the hashed inputs
    outdigs = Vector{Int8}(undef, ndigs)  # Digits of ID
    
    # Sum up binary digits to get the digit in this ID position
    n = 64 ÷ ndigs
    for i=0:ndigs-1
        outdigs[i+1] = sum(hashdigs[n*i+1:n*(i+1)]) % 10
    end

    return string(sum(outdigs[i]*10^(i-1) for i in 1:ndigs))
end
