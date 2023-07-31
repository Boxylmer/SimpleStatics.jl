
"Create a pair of items whose order is independent in hashes, equality, and comparisons."
struct UnorderedPair; a::Int32; b::Int32; end
Base.hash(x::UnorderedPair, h::UInt) = hash(x.a < x.b ? (x.a, x.b) : (x.b, x.a), h)
Base.isequal(x::UnorderedPair, y::UnorderedPair) = x.a == y.a ? x.b == y.b : (x.a == y.b && x.b == y.a)
higher(x::UnorderedPair) = max(x.a, x.b)
lower(x::UnorderedPair) = min(x.a, x.b)
Base.:(==)(x::UnorderedPair, y::UnorderedPair) = isequal(x, y)
function Base.iterate(pair::UnorderedPair, state=1)
    if state == 1
        return (pair.a, state+1)
    elseif state == 2
        return (pair.b, nothing)
    end
end
Base.length(::UnorderedPair) = 2



struct Vector2D{T}
    x::T
    y::T
end
# Addition
Base.:+(u::Vector2D, v::Vector2D) = Vector2D(u.x + v.x, u.y + v.y)

# Subtraction
Base.:-(u::Vector2D, v::Vector2D) = Vector2D(u.x - v.x, u.y - v.y)

# Scalar multiplication (left and right)
Base.:*(a, v::Vector2D) = Vector2D(a * v.x, a * v.y)
Base.:*(v::Vector2D, a) = Vector2D(v.x * a, v.y * a)

function Base.iterate(v::Vector2D, state=1)
    if state == 1
        return (v.x, state+1)
    elseif state == 2
        return (v.y, nothing)
    end
end
Base.length(::Vector2D) = 2

norm(v::Vector2D) = sqrt(v.x^2 + v.y^2)

function unit_vector(v::Vector2D)
    mag = sqrt(v.x^2 + v.y^2)
    return Vector2D(v.x / mag, v.y / mag)
end

Base.eltype(::Vector2D{T}) where T = T 