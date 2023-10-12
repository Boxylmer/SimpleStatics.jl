struct StaticMaterial{AT, MT, YT, DT}
    area::AT # in square meters
    modulus::MT # in pa or netwon / meter^2
    yield::YT # pa or netwon / meter^2
    density::DT  # g/cm3
end

"""
    StaticMaterial(area, modulus, [yield=Inf], [density=0.0])

Create a new material for use in a statics problem with a...
    - area: Cross sectional area in square meters. 
    - modulus: Elastic (Youngs) modulus in pa i.e., N/m^2.
    - yield: Yield strength (same units as modulus), used primarily when approximating structure failure and safety ratings.
    - density: Material density in g/cm3, used primarily to calculate self-loading and structure mass. 
"""
StaticMaterial(a, m) = StaticMaterial(a, m, Inf, 0.0)

"Calculate how much a given length of the material weighs, in Newtons. Optionally, gravity can be specified in m/s2."
weight(m::StaticMaterial, length_m::Number; g=9.81) = mass(m, length_m) * g  # kg * m/s2 = N

"Calculate the mass of a material in kg given its length in meters."
mass(m::StaticMaterial, length_m::Number) = m.area * length_m * m.density * 1e6 / 1000 # m * m2 * g/cm3 * 1e6cm3/m3 * 1kg/1000g -> kg

module Materials
    import ..StaticMaterial
    # http://web.mit.edu/16.20/homepage/3_Constitutive/Constitutive_files/module_3_with_solutions.pdf
    "Ideal material with no mass, a massive cross section, and unrealistically high strength."
    PerfectMaterial()::StaticMaterial = StaticMaterial(1.0, 1e12)

    "Create a tungsten member with a given cross section (in m^2)."
    Tungsten(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 410e9, 0.75e9, 13.4)

    "Create a stainless steel member with a given cross section (in m^2)."
    StainlessSteel(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 195e9, 0.215e9, 7.6)
    # https://blog.thepipingmart.com/metals/mild-carbon-steel-properties-an-overview

    "Create a mild steel member with a given cross section (in m^2)."
    MildSteel(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 196e9, 245e6, 7.8)


    # https://amesweb.info/Materials/Youngs-Modulus-of-Wood.aspx
    "Create a PVC member with a given cross section (in m^2)."
    PVC(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 0.0065e9, 15e6, 1.5)

    # https://core.ac.uk/download/pdf/153414189.pdf
    # https://www.engineeringtoolbox.com/timber-mechanical-properties-d_1789.html
    "Create a pine member with a given cross section (in m^2)."
    Pine(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 4.141e9, 80e6, 0.8778118)

    "Create a standard pine 2x4 member."
    Pine2x4() = Pine(0.00338709)

    """
        SquareTubing(material::Function, outer_length_m, thickness_m)

    Create a square tubing material from another raw material, i.e., a material function that accepts only an area.
    E.g., 
        ```
        large_steel_beam = Materials.SquareTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000)  # 1.5" steel tubing with 14 gauge thickness
        small_steel_beam = Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000)  # 1.0" steel tubing with 16 gauge thickness
        ```
    """
    SquareTubing(material::Function, outer_length_m, thickness_m) = material(square_tubing_area(outer_length_m, thickness_m))
    
    """
    CircularTubing(material::Function, outer_diameter_m, thickness_m)

        Create a pipe-like material from another raw material, i.e., a material function that accepts only an area.
        E.g., 
            ```
            large_steel_pipe = Materials.CircularTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000)  # 1.5" diameter steel pipe with 14 gauge thickness
            small_steel_pipe = Materials.CircularTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000)  # 1.0" diameter steel pipe with 16 gauge thickness
            ```
    """
    CircularTubing(material::Function, outer_diameter_m, thickness_m) = material(circular_tubing_area(outer_diameter_m, thickness_m))

    square_tubing_area(outer_length_m, thickness_m) = outer_length_m^2 - (outer_length_m - 2 * thickness_m)^2
    circular_tubing_area(outer_diameter_m, thickness_m) = (outer_diameter_m^2 - (outer_diameter_m - 2*thickness_m)^2) * pi / 4 

end