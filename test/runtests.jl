using SimpleStatics
using Test

using Measurements # todo add this to deps for docs


@testset "SimpleStatics.jl" begin
    # Matrix Analysis of Structures, chapter 4, truss in example 4.2
    function example_setup()
        s = StaticSetup(Float64)
        j1 = add_joint!(s, 0.0, 0.0)
        j2 = add_joint!(s, 16.0 * 12, 0.0 * 12)
        j3 = add_joint!(s, 16.0 * 12, 12.0 * 12)
        j4 = add_joint!(s, 32.0 * 12, 0.0 * 12)
        j5 = add_joint!(s, 32.0 * 12, 12.0 * 12)

        m = StaticMaterial(10., 30000)
        m1 = add_member!(s, j1, j3, m)
        m2 = add_member!(s, j1, j2, m)
        m3 = add_member!(s, j2, j3, m)
        m4 = add_member!(s, j3, j5, m)
        m5 = add_member!(s, j3, j4, m)
        m6 = add_member!(s, j2, j5, m)
        m7 = add_member!(s, j2, j4, m)
        m8 = add_member!(s, j4, j5, m)

        set_constraint!(s, j1, AnchorConstraint())
        set_constraint!(s, j4, AnchorConstraint())
        set_force!(s, j2, 0., -100.)
        set_force!(s, j5, 50., 0.)
        return s
    end

    function example_setup_constraints()  # setup to test all constraints
        s = StaticSetup(Float64)
        j1 = add_joint!(s, 0.0, 0.0)
        j2 = add_joint!(s, 1., 0.)
        j3 = add_joint!(s, 2, 0)
        j4 = add_joint!(s, 1, 1)

        m1 = add_member!(s, j1, j2)
        m2 = add_member!(s, j1, j4)
        m3 = add_member!(s, j2, j4)
        m4 = add_member!(s, j2, j3)
        m5 = add_member!(s, j4, j3)

        set_constraint!(s, j1, AnchorConstraint())
        set_constraint!(s, j3, XRollerConstraint())
        set_constraint!(s, j4, YRollerConstraint())

        set_force!(s, j4, 0., -100.)
        return s
    end

    @testset "HelperFunctions.jl" begin
        # unordered pair
        p1 = SimpleStatics.UnorderedPair(1, 2)
        p2 = SimpleStatics.UnorderedPair(2, 1)
        @test p1 == p2

        @test SimpleStatics.higher(p1) == 2
        @test SimpleStatics.lower(p1) == 1

        @test length(p1) == length(p2) == 2

        # Vector2D
        v1 = SimpleStatics.Vector2D(2, 3)
        v2 = SimpleStatics.Vector2D(-4, 5)
        @test v1 + v2 == SimpleStatics.Vector2D(-2, 8)
        @test v1 - v2 == SimpleStatics.Vector2D(6, -2)
        @test v1 * 1 == v1
        @test 4 * v2 == SimpleStatics.Vector2D(-16, 20)

        added = 0
        for val in v1
            added += val
        end
        @test added == 5 

        @test length(v1) == length(v2) == 2

        @test SimpleStatics.unit_vector(v1) ≈ SimpleStatics.Vector2D(0.5547001962252291, 0.8320502943378437)
        @test SimpleStatics.norm(v1) ≈ 3.605551275463989

        
        @test SimpleStatics.Vector2D(1.0, 2) == SimpleStatics.Vector2D(1, 2.0)
        a = SimpleStatics.Vector2D(1 ± 0.1, 2 ± 0.1)
        b = SimpleStatics.Vector2D(5, 6)
        @test (a + b).x.err == SimpleStatics.Vector2D(6 ± 0.1, 8 ± 0.1).x.err
    end
    
    @testset "StaticConstraint.jl" begin
        @test typeof(NoConstraint()) <: SimpleStatics.SimpleConstraint
        @test typeof(AnchorConstraint()) <: SimpleStatics.SimpleConstraint
        @test typeof(XRollerConstraint()) <: SimpleStatics.SimpleConstraint
        @test typeof(YRollerConstraint()) <: SimpleStatics.SimpleConstraint
    end

    @testset "StaticMaterial.jl" begin
        m1 = Materials.PerfectMaterial()
        m2 = Materials.Tungsten(1)
        @test m1.modulus > m2.modulus # a perfect material is more resistant to strain than tungsten, the strongest metal. 

        large_steel_beam = Materials.SquareTubing(Materials.MildSteel, 1.5 * 0.0381, 1.897 / 1000)  # 1.5" steel tubing with 14 gauge thickness
        small_steel_beam = Materials.SquareTubing(Materials.MildSteel, 1.0 * 0.0381, 1.518 / 1000)  # 1.0" steel tubing with 16 gauge thickness
        @test weight(large_steel_beam, 1) > weight(small_steel_beam, 1)

        
    end

    @testset "StaticSetup.jl" begin
        s = StaticSetup()

        #       ↓ 50N
        # 2 --- 3  ← 5N
        # |   /
        # | /
        # 1 

        j1 = add_joint!(s, 0, 0, AnchorConstraint())
        j2 = add_joint!(s, 0, 1)
        j3 = add_joint!(s, 1, 1, YRollerConstraint())

        m1 = add_member!(s, j1, j2)
        m2 = add_member!(s, j2, j3)
        m3 = add_member!(s, j3, j1)

        @test_throws(ArgumentError, add_member!(s, j1, j2))
        @test_throws(ArgumentError, add_member!(s, j1, j1))

        @test j2 == 2
        @test m2 == 2

        @test n_joints(s) == 3
        @test n_members(s) == 3
        @test joint_ids(s) == [1, 2, 3]
        @test member_ids(s) == [1, 2, 3]

        set_force!(s, j3, -5, -50)

        s = example_setup()
        gsm = global_stiffness_matrix(s)

        m = SimpleStatics.constrained_dof_array_mapping(s)
        gsm_constrained = SimpleStatics.constrained_array(gsm, m)
        @test gsm_constrained[2, 4] ≈ -2083 - 1/3


        @test member_angle(s, 2) == 0 
    end

    @testset "Solvers.jl" begin
        setup = example_setup()

        d = solve_displacements(setup)
        @test d[2] ≈ SimpleStatics.Vector2D{Float64}(0.01460 + 2e-5/3, -0.10464041 - 2e-8/3)

        e = equilibrium_positions(setup, d)
        @test d[1] + setup.positions[1] == e[1]
        @test d[end] + setup.positions[end] == e[end]

        s = solve_member_stresses(setup, d)
        @test s[3] == s[end] == 0 # zero force members
        @test s[1] ≈ (52 + 1/12)


        r = solve_reaction_forces(setup, d)
        @test Base.rtoldefault(sum(setup.forces) + sum(r), SimpleStatics.Vector2D(0.0, 0.0)) ≈ 1.0587911840678754e-22 
        @test isapprox((sum(setup.forces) + sum(r)), SimpleStatics.Vector2D(0.0, 0.0), atol=1e-10)

        # Edge Case 1: Test vertical stresses actually working
        s2 = example_setup()
        set_force!(s2, 3, 0, -10)
        d = solve_displacements(setup)
        s = solve_member_stresses(setup, d)

        # Edge Case 2: All constraints used at once
        s3 = example_setup_constraints()
        d = solve_displacements(s3)
        s = solve_member_stresses(s3, d)
        @test s[2] != 0
        @test s[1] ≈ s[4]

        

    end

    @testset "Visual.jl" begin
        output_folder = joinpath(@__DIR__, "test_output")

        setup = example_setup()
        plot_setup(setup, joinpath(output_folder, "basic"))

        d = solve_displacements(setup)
        s = solve_member_stresses(setup, d)
        r = solve_reaction_forces(setup, d)

        plot_setup(setup, 
            joinpath(output_folder, "dsr"), 
            padding = 1.0,
            displacements = d,
            stresses = s,
            reactions = r
        )

        # test out other visual elements like roller constraints
        sx = example_setup()
        set_constraint!(sx, 1, XRollerConstraint())
        set_constraint!(sx, 5, YRollerConstraint())
        plot_setup(sx, joinpath(output_folder, "none-roller"))
        

        # test case where stresses exist but displacements do not
        setup = example_setup()
        plot_setup(setup, joinpath(output_folder, "basic"))

        d = solve_displacements(setup)
        s = solve_member_stresses(setup, d)
        r = solve_reaction_forces(setup, d)

        plot_setup(setup, 
            joinpath(output_folder, "dr"),
            displacements = d,
            reactions = r
        )
    end


end