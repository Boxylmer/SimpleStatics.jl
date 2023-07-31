using Revise
using SimpleStatics
using Test

@testset "SimpleStatics.jl" begin
    
    @testset "HelperFunctions.jl" begin
        # unordered pair
        p1 = SimpleStatics.UnorderedPair(1, 2)
        p2 = SimpleStatics.UnorderedPair(2, 1)
        @test p1 == p2



        # Vector2D
    end


    @testset "Base.jl" begin
        # constraints

        # materials

        # StaticSetup
        s = StaticSetup()
        # 2 --- 3
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
    end



    
end
