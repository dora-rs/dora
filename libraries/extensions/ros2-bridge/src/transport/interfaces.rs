//! Which local networks dora's DDS participant advertises (#4227).
//!
//! RustDDS announces a unicast locator for every IPv4 interface, including
//! `docker0`, VPN and link-local ones. A ROS 2 node on another host may send
//! to one of those it cannot reach, and its data silently never arrives.
//! [`INTERFACES_ENV`] restricts the participant to the given addresses.

use std::net::Ipv4Addr;

/// Comma-separated local IPv4 addresses to advertise and listen on.
pub const INTERFACES_ENV: &str = "DORA_ROS2_DDS_INTERFACES";

/// One IPv4 address of a local interface.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct LocalAddr {
    pub name: String,
    pub ip: Ipv4Addr,
    pub loopback: bool,
    pub multicast: bool,
    pub up: bool,
}

/// The host's IPv4 addresses, from the same enumeration RustDDS uses.
pub(crate) fn local_addrs() -> Vec<LocalAddr> {
    netdev::get_interfaces()
        .into_iter()
        .flat_map(|iface| {
            let (loopback, multicast, up) =
                (iface.is_loopback(), iface.is_multicast(), iface.is_up());
            iface.ipv4.into_iter().map(move |net| LocalAddr {
                name: iface.name.clone(),
                ip: net.addr(),
                loopback,
                multicast,
                up,
            })
        })
        .collect()
}

fn list(addrs: impl IntoIterator<Item = impl std::borrow::Borrow<LocalAddr>>) -> String {
    addrs
        .into_iter()
        .map(|a| format!("{} {}", a.borrow().name, a.borrow().ip))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Parse the value of [`INTERFACES_ENV`]: `None` when it is unset or blank.
pub(crate) fn parse(value: Option<&str>) -> Result<Option<Vec<Ipv4Addr>>, String> {
    let Some(value) = value.filter(|v| !v.trim().is_empty()) else {
        return Ok(None);
    };
    value
        .split(',')
        .map(str::trim)
        .filter(|entry| !entry.is_empty())
        .map(|entry| {
            entry.parse::<Ipv4Addr>().map_err(|_| {
                format!(
                    "{INTERFACES_ENV}: `{entry}` is not an IPv4 address; list the local IPv4 \
                     addresses to use, separated by commas"
                )
            })
        })
        .collect::<Result<Vec<_>, _>>()
        .and_then(|addrs| {
            if addrs.is_empty() {
                Err(format!(
                    "{INTERFACES_ENV} lists no address; unset it to use every interface"
                ))
            } else {
                Ok(Some(addrs))
            }
        })
}

/// Every requested address must belong to this host: RustDDS would otherwise
/// drop it silently and the participant could end up invisible to peers.
pub(crate) fn check(requested: &[Ipv4Addr], local: &[LocalAddr]) -> Result<(), String> {
    let missing: Vec<String> = requested
        .iter()
        .filter(|ip| !local.iter().any(|a| a.ip == **ip))
        .map(Ipv4Addr::to_string)
        .collect();
    if missing.is_empty() {
        return Ok(());
    }
    Err(format!(
        "{INTERFACES_ENV} lists {}, which this host does not have; its IPv4 addresses are: {}",
        missing.join(", "),
        list(local),
    ))
}

/// The networks to restrict the participant to, from [`INTERFACES_ENV`].
pub(crate) fn selected(
    value: Option<&str>,
    local: &[LocalAddr],
) -> Result<Option<Vec<Ipv4Addr>>, String> {
    let Some(requested) = parse(value)? else {
        return Ok(None);
    };
    check(&requested, local)?;
    Ok(Some(requested))
}

/// Problems with selected addresses that RustDDS would only log below dora's
/// default level.
pub(crate) fn selection_warnings(requested: &[Ipv4Addr], local: &[LocalAddr]) -> Vec<String> {
    local
        .iter()
        .filter(|a| requested.contains(&a.ip))
        .filter_map(|a| {
            let addr = format!("{} {}", a.name, a.ip);
            if !a.up {
                Some(format!(
                    "{INTERFACES_ENV}: {addr} is down, so ROS 2 peers cannot reach dora there"
                ))
            } else if !a.multicast {
                Some(format!(
                    "{INTERFACES_ENV}: {addr} cannot send multicast, so dora joins no \
                     discovery group there; ROS 2 nodes on other hosts will not find it"
                ))
            } else {
                None
            }
        })
        .collect()
}

/// What to tell the user when [`INTERFACES_ENV`] is unset and the participant
/// will advertise more than one non-loopback address.
pub(crate) fn unrestricted_warning(local: &[LocalAddr]) -> Option<String> {
    let advertised: Vec<&LocalAddr> = local.iter().filter(|a| !a.loopback).collect();
    if advertised.len() < 2 {
        return None;
    }
    Some(format!(
        "the DDS participant advertises {} network addresses ({}); a ROS 2 node on another host \
         may send to one it cannot reach, and then its data never arrives. Set \
         {INTERFACES_ENV}=<address> to advertise only the network your ROS 2 peers are on",
        advertised.len(),
        list(advertised),
    ))
}

/// Print [`unrestricted_warning`] once per process.
pub(crate) fn report_unrestricted(local: &[LocalAddr]) {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        if let Some(message) = unrestricted_warning(local) {
            eprintln!("warning: {message}");
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn addr(name: &str, ip: &str) -> LocalAddr {
        let ip: Ipv4Addr = ip.parse().unwrap();
        LocalAddr {
            name: name.into(),
            ip,
            loopback: ip.is_loopback(),
            multicast: !ip.is_loopback(),
            up: true,
        }
    }

    fn host() -> Vec<LocalAddr> {
        vec![
            addr("lo", "127.0.0.1"),
            addr("eno1", "192.168.1.231"),
            addr("docker0", "172.17.0.1"),
            addr("tailscale0", "100.107.163.59"),
        ]
    }

    #[test]
    fn an_unset_or_blank_value_means_no_restriction() {
        assert_eq!(parse(None), Ok(None));
        assert_eq!(parse(Some("")), Ok(None));
        assert_eq!(parse(Some("  ")), Ok(None));
    }

    #[test]
    fn a_list_of_addresses_is_parsed_with_whitespace_and_empty_entries_ignored() {
        assert_eq!(
            parse(Some(" 192.168.1.231 ,10.0.0.5,")),
            Ok(Some(vec![
                Ipv4Addr::new(192, 168, 1, 231),
                Ipv4Addr::new(10, 0, 0, 5)
            ]))
        );
    }

    #[test]
    fn an_interface_name_or_ipv6_address_is_rejected_with_the_entry() {
        for entry in ["eno1", "::1", "192.168.1"] {
            let error = parse(Some(entry)).unwrap_err();
            assert!(error.contains(INTERFACES_ENV), "{error}");
            assert!(error.contains(&format!("`{entry}`")), "{error}");
            assert!(error.contains("IPv4 address"), "{error}");
        }
    }

    #[test]
    fn local_addresses_pass_the_check() {
        assert_eq!(check(&[Ipv4Addr::new(192, 168, 1, 231)], &host()), Ok(()));
    }

    #[test]
    fn an_address_that_is_not_local_is_an_error_listing_the_local_ones() {
        let error = check(
            &[Ipv4Addr::new(192, 168, 1, 231), Ipv4Addr::new(192, 0, 2, 1)],
            &host(),
        )
        .unwrap_err();
        assert!(error.contains("lists 192.0.2.1, which"), "{error}");
        assert!(error.contains("eno1 192.168.1.231"), "{error}");
        assert!(error.contains("docker0 172.17.0.1"), "{error}");
    }

    #[test]
    fn selected_combines_parse_and_check() {
        assert_eq!(selected(None, &host()), Ok(None));
        assert_eq!(
            selected(Some("192.168.1.231"), &host()),
            Ok(Some(vec![Ipv4Addr::new(192, 168, 1, 231)]))
        );
        assert!(selected(Some("192.0.2.1"), &host()).is_err());
        assert!(selected(Some("eno1"), &host()).is_err());
    }

    #[test]
    fn one_non_loopback_address_needs_no_warning() {
        let local = vec![addr("lo", "127.0.0.1"), addr("eno1", "192.168.1.231")];
        assert_eq!(unrestricted_warning(&local), None);
    }

    #[test]
    fn several_non_loopback_addresses_are_listed_with_the_fix() {
        let message = unrestricted_warning(&host()).unwrap();
        assert!(message.contains("eno1 192.168.1.231"), "{message}");
        assert!(message.contains("docker0 172.17.0.1"), "{message}");
        assert!(message.contains("tailscale0 100.107.163.59"), "{message}");
        assert!(!message.contains("127.0.0.1"), "{message}");
        assert!(message.contains(INTERFACES_ENV), "{message}");
    }

    #[test]
    fn a_value_with_only_separators_is_an_error() {
        for value in [",", " , ,"] {
            let error = parse(Some(value)).unwrap_err();
            assert!(error.contains(INTERFACES_ENV), "{error}");
            assert!(error.contains("no address"), "{error}");
        }
    }

    #[test]
    fn a_usable_selection_needs_no_warning() {
        assert!(selection_warnings(&[Ipv4Addr::new(192, 168, 1, 231)], &host()).is_empty());
    }

    #[test]
    fn an_address_without_multicast_is_warned_about() {
        let warnings = selection_warnings(&[Ipv4Addr::LOCALHOST], &host());
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("lo 127.0.0.1"), "{warnings:?}");
        assert!(warnings[0].contains("multicast"), "{warnings:?}");
    }

    #[test]
    fn an_address_on_a_down_interface_is_warned_about() {
        let mut local = host();
        local[1].up = false;
        let warnings = selection_warnings(&[Ipv4Addr::new(192, 168, 1, 231)], &local);
        assert_eq!(warnings.len(), 1, "{warnings:?}");
        assert!(warnings[0].contains("eno1 192.168.1.231"), "{warnings:?}");
        assert!(warnings[0].contains("down"), "{warnings:?}");
    }

    #[test]
    fn local_addrs_includes_loopback() {
        assert!(
            local_addrs()
                .iter()
                .any(|a| a.loopback && a.ip == Ipv4Addr::LOCALHOST)
        );
    }
}
