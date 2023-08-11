abstract type StaticConstraint end

abstract type SimpleConstraint <: StaticConstraint end

"Default constraint that restricts no movement."
struct NoConstraint <: SimpleConstraint end
"Completely restrictive constraint that restricts all movement. "
struct AnchorConstraint <: SimpleConstraint end
"Restricts movement along the vertical axis only, permitting horizontal movement."
struct XRollerConstraint <: SimpleConstraint end
"Restricts movement along the vertical axis only, permitting horizontal movement."
struct YRollerConstraint <: SimpleConstraint end
