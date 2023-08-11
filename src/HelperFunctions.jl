
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
Vector2D(x, y) = Vector2D(promote(x, y)...)

# Addition
Base.:+(u::Vector2D, v::Vector2D) = Vector2D(u.x + v.x, u.y + v.y)

# Subtraction
Base.:-(u::Vector2D, v::Vector2D) = Vector2D(u.x - v.x, u.y - v.y)

# Scalar multiplication (left and right)
Base.:*(a, v::Vector2D) = Vector2D(a * v.x, a * v.y)
Base.:*(v::Vector2D, a) = Vector2D(v.x * a, v.y * a)

Base.zero(::Vector2D{T}) where T = Vector2D(zero(T), zero(T))
Base.zero(::Type{Vector2D{T}}) where T = Vector2D(zero(T), zero(T))
Base.zero(::Type{Vector2D}) = Vector2D(zero(Float64), zero(Float64))

Base.one(::Vector2D{T}) where T = Vector2D(one(T), one(T))
Base.one(::Type{Vector2D{T}}) where T = Vector2D(one(T), one(T))
Base.one(::Type{Vector2D}) = Vector2D(one(Float64), one(Float64))


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

function Base.isapprox(a::Vector2D, b::Vector2D; atol=0.0, rtol=Base.rtoldefault(a,b))
    x_approx = abs(a.x - b.x) <= atol + rtol * abs(b.x)
    y_approx = abs(a.y - b.y) <= atol + rtol * abs(b.y)
    return x_approx && y_approx
end

function Base.rtoldefault(a::Vector2D{T}, b::Vector2D{T}) where T
    # Get the machine epsilon for type T
    epsilon = eps(T)

    # Compute relative tolerances based on the magnitude of a and b
    ra = maximum([abs(a.x), abs(a.y)])
    rb = maximum([abs(b.x), abs(b.y)])

    # Return the relative tolerance based on the larger of the two magnitudes
    return sqrt(epsilon) * maximum([ra, rb])
end