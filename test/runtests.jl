using SimpleStatics
using Test



@testset "SimpleStatics.jl" begin
    
    @testset "HelperFunctions.jl" begin
        # unordered pair
        p1 = SimpleStatics.UnorderedPair(1, 2)
        p2 = SimpleStatics.UnorderedPair(2, 1)
        @test p1 == p2



    end


    @testset "Base.jl" begin
        # constraints

        # materials

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


    end


end
nothing