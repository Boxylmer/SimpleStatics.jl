using SimpleStatics
using Test



@testset "SimpleStatics.jl" begin
    
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

    end


    @testset "Base.jl" begin
        # constraints
        @test typeof(NoConstraint()) <: SimpleStatics.SimpleConstraint
        @test typeof(AnchorConstraint()) <: SimpleStatics.SimpleConstraint
        @test typeof(XRollerConstraint()) <: SimpleStatics.SimpleConstraint
        @test typeof(YRollerConstraint()) <: SimpleStatics.SimpleConstraint
        


        # materials
        m1 = PerfectMaterial()
        m2 = Tungsten(1)
        @test m1.modulus > m2.modulus # a perfect material is more resistant to strain than tungsten, the strongest metal. 


        # StaticSetup
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

        @test j2 == 2
        @test m2 == 2

        @test n_joints(s) == 3
        @test n_members(s) == 3
        @test joint_ids(s) == [1, 2, 3]
        @test member_ids(s) == [1, 2, 3]

        set_force!(s, j3, -5, -50)
    end

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

    @testset "Solvers.jl" begin
        s = example_setup()

        gsm = global_stiffness_matrix(s)

        m = SimpleStatics.constrained_dof_array_mapping(s)
        gsm_constrained = SimpleStatics.constrained_array(gsm, m)
        @test gsm_constrained[2, 4] ≈ -2083 - 1/3

        d = solve_displacements(s)
       
        @test d[2] ≈ SimpleStatics.Vector2D{Float64}(0.01460 + 2e-5/3, -0.10464041 - 2e-8/3)
    end


end
nothing