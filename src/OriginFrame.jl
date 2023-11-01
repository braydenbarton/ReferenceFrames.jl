"""
A special kind of inertial frame which must be the highest point in a ReferenceFrame hierarchy. All reference frames 
will descend from an `OriginFrame` in some way. If multiple unrelated frame hierarchies are required, a new origin must 
be declared for each.

# Properties
- `ID`: The string ID of the frame. Used for display, is not necessarily unique
"""
struct OriginFrame <: ReferenceFrame 
    ID::String
end

_framename(::OriginFrame) = "OriginFrame"

getposition(::OriginFrame, args...) = [0, 0, 0]
getvelocity(::OriginFrame, args...) = [0, 0, 0]
getacceleration(::OriginFrame, args...) = [0, 0, 0]

getquaternion(::OriginFrame, args...) = [1, 0, 0, 0]
getomega(::OriginFrame, args...) = zeros(3)
getalpha(::OriginFrame, args...) = zeros(3)

"When called on an `OriginFrame` object, return the object itself"
getfamily(frame::OriginFrame) = [frame]

"An OriginFrame's parent is itself"
getparent(frame::OriginFrame) = frame