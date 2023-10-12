using SimpleStatics
using Documenter

DocMeta.setdocmeta!(SimpleStatics, :DocTestSetup, :(using SimpleStatics); recursive=true)

makedocs(;
    modules=[SimpleStatics],
    authors="Will <William.Joseph.Box@gmail.com> and contributors",
    repo="https://github.com/Boxylmer/SimpleStatics.jl/blob/{commit}{path}#{line}",
    sitename="SimpleStatics.jl",
    format=Documenter.HTML(;
        prettyurls=get(ENV, "CI", "false") == "true",
        canonical="https://github.com/Boxylmer/SimpleStatics.jl",
        edit_link="main",
        assets=String[],
    ),
    pages=[
        "Home" => "index.md",
        "Example Workflow" => "example workflow.md",
        "Examples" => [
            "Example 1" => "examples/example1.md"
        ],
        "Reference" => "reference.md",
    ],
)

deploydocs(;
    repo="github.com/Boxylmer/SimpleStatics.jl",
    devbranch="main",
)
