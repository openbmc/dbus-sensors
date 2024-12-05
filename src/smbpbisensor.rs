use anyhow::Context;
use clap::Parser;
use i2cdev::core::I2CDevice;
use i2cdev::linux::LinuxI2CDevice;

const MAX_POST_BOX_SIZE: usize = std::mem::size_of::<u64>();

/// Smbus post-box interface sensor management
#[derive(Debug, clap::Parser)]
struct Args {
    #[command(subcommand)]
    subcommand: Subcommand,
}

#[derive(Debug, clap::Subcommand)]
enum Subcommand {
    /// Read a post-box
    Read {
        /// Post-box interface I2C bus index
        #[arg(long)]
        bus: u8,

        /// Post-box interface I2C address
        #[arg(long)]
        address: u16,

        /// Post-box offset
        #[arg(long)]
        offset: u16,

        /// Post-box size
        #[arg(long)]
        size: usize,
    },
}

/// This reads a post-box from an NVIDIA smbus post-box interface.
///
/// NOTE: This protocol is *not* implemented via standard smbus commands.
fn smbus_read_post_box<T>(i2c: &mut T, offset: u16, out: &mut [u8]) -> anyhow::Result<()>
where
    T: I2CDevice,
    <T as I2CDevice>::Error: std::error::Error + Send + Sync + 'static,
{
    let mut buf = [0_u8; std::mem::size_of::<u16>()];
    let offset = if offset <= u8::MAX as u16 {
        // 8-bit address is transmitted directly
        buf[0] = offset as u8;
        &buf[..=0]
    } else {
        // >8-bit address is transmitted in big-endian byte order.
        buf = offset.to_be_bytes();
        &buf[..]
    };
    i2c.write(offset)
        .context("Unable to write post-box offset")?;
    i2c.read(out).context("Unable to read post-box data")?;
    Ok(())
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    match args.subcommand {
        Subcommand::Read {
            bus,
            address,
            offset,
            size,
        } => {
            let bus_path = format!("/dev/i2c-{bus}");
            let mut i2c = LinuxI2CDevice::new(&bus_path, address)
                .with_context(|| format!("Unable to open {bus_path} @{address}"))?;

            let mut value = [0_u8; MAX_POST_BOX_SIZE];
            anyhow::ensure!(
                size <= MAX_POST_BOX_SIZE,
                "Maximum post-box size if {MAX_POST_BOX_SIZE} bytes"
            );

            smbus_read_post_box(&mut i2c, offset, &mut value[..size])
                .with_context(|| format!("Unable to read post-box at +{offset}, size={size}"))?;

            println!("{value:02x?}");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use fixed::types::I24F8;
    use i2cdev::mock::MockI2CDevice;

    #[test]
    fn test_smbus_read_post_box() -> anyhow::Result<()> {
        // This mock data is derived from a real HMC temperature sensor reading.
        let mut i2c = MockI2CDevice::new();
        i2c.regmap.write_regs(240, &[0x2f, 0x20, 0x00, 0x00]);

        let mut buf = [0_u8; 4];
        smbus_read_post_box(&mut i2c, 240, &mut buf)?;
        println!("{buf:02x?}");
        let temperature = I24F8::from_le_bytes(buf);
        println!("{temperature}");
        assert_eq!(temperature, 32.18359375);

        Ok(())
    }
}
