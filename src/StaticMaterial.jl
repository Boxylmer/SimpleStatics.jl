struct StaticMaterial{AT, MT, YT, DT}
    area::AT # in square meters
    modulus::MT # in pa or netwon / meter^2
    yield::YT # pa or netwon / meter^2
    density::DT  # g/cm3
end

StaticMaterial(a, m) = StaticMaterial(a, m, Inf, 0.0)

"Calculate how much a given length of the material weighs, in Newtons. Optionally, gravity can be specified in m/s2."
weight(m::StaticMaterial, length_m::Number; g=9.81) = mass(m, length_m) * g  # kg * m/s2 = N

"Calculate the mass of a material in kg given its length in meters."
mass(m::StaticMaterial, length_m::Number) = m.area * length_m * m.density * 1e6 / 1000 # m * m2 * g/cm3 * 1e6cm3/m3 * 1kg/1000g -> kg

module Materials
    import ..StaticMaterial
    # http://web.mit.edu/16.20/homepage/3_Constitutive/Constitutive_files/module_3_with_solutions.pdf
    PerfectMaterial()::StaticMaterial = StaticMaterial(1.0, 1e12)
    Tungsten(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 410e9, 0.75e9, 13.4)
    StainlessSteel(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 195e9, 0.215e9, 7.6)
    # https://blog.thepipingmart.com/metals/mild-carbon-steel-properties-an-overview
    MildSteel(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 196e9, 245e6, 7.8)


    # https://amesweb.info/Materials/Youngs-Modulus-of-Wood.aspx
    PVC(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 0.0065e9, 15e6, 1.5)
    # https://core.ac.uk/download/pdf/153414189.pdf
    # https://www.engineeringtoolbox.com/timber-mechanical-properties-d_1789.html
    Pine(area_m2::Number)::StaticMaterial = StaticMaterial(area_m2, 4.141e9, 80e6, 0.8778118)
    Pine2x4() = Pine(0.00338709)

    SquareTubing(material::Function, outer_length_m, thickness_m) = material(square_tubing_area(outer_length_m, thickness_m))
    CircularTubing(material::Function, outer_diameter_m, thickness_m) = material(circular_tubing_area(outer_diameter_m, thickness_m))

    square_tubing_area(outer_length_m, thickness_m) = outer_length_m^2 - (outer_length_m - 2 * thickness_m)^2
    circular_tubing_area(outer_diameter_m, thickness_m) = (outer_diameter_m^2 - (outer_diameter_m - 2*thickness_m)^2) * pi / 4 

end