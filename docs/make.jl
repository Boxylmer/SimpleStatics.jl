using SimpleStatics
using Documenter

DocMeta.setdocmeta!(SimpleStatics, :DocTestSetup, :(using SimpleStatics); recursive=true)

makedocs(;
    modules=[SimpleStatics],
    authors="Will <sypheren@gmail.com> and contributors",
    repo="https://github.com/Boxylmer/SimpleStatics.jl/blob/{commit}{path}#{line}",
    sitename="SimpleStatics.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://Boxylmer.github.io/SimpleStatics.jl",
        edit_link="master",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
    ],
)

deploydocs(;
    repo="github.com/Boxylmer/SimpleStatics.jl",
    devbranch="master",
)
