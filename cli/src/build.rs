use clap::CommandFactory;
use clap_complete::{generate_to, shells::Bash};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let outdir = std::path::Path::new("completions");
    let mut cmd = cli::Cli::command();
    
    generate_to(
        Bash,
        &mut cmd,
        "rustle",
        outdir,
    )?;

    Ok(())
}