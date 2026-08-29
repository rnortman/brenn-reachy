//! The port numbers on the driver seam, as three artifacts state them.
//!
//! `crates/reachy-motord/src/ports.rs` names every subject of the seam and
//! asserts the numbers are disjoint. It cannot see the compositions: `robot.clk`
//! and `robot_host.clk` restate the same numbers as bare literals, and a `const`
//! assert in Rust says nothing about a `.clk` file. This is the join.
//!
//! Two claims. The seam the control box binds is the seam the driver names, so a
//! subject added on one side and not the other fails here rather than as a
//! datagram decoded under the wrong schema. And every port bound anywhere in the
//! host composition is bound once: that composition puts the control box's
//! incoming sockets and the simulated plant's on one loopback, so two of them
//! sharing a number is two processes binding one port -- a launcher failure on a
//! workstation, which is what happened the last time a subject was added to the
//! seam and which nothing but a person caught.

use reachy_motord::ports;

/// The environment variables naming the two compositions, relative to the
/// runfiles root, which is a test's working directory.
const ROBOT_CLK: &str = "ROBOT_CLK";
const ROBOT_HOST_CLK: &str = "ROBOT_HOST_CLK";

/// One socket declaration: what it is called, which port it names, and whether
/// it binds that port or sends to it.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Socket {
    name: String,
    port: u16,
    incoming: bool,
}

/// The contents of the file `name` points at.
///
/// Panics rather than answers: a missing runfile is a broken test target, not a
/// case.
fn runfile(name: &str) -> String {
    let path = std::env::var(name).unwrap_or_else(|_| {
        panic!(
            "{name} is unset: the test target has to name the file beside the data attribute that \
             supplies it"
        )
    });
    std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("{name} names {path}, which does not read: {error}"))
}

/// Every `udp_socket` the composition in `name` declares.
///
/// A line reader rather than a parse of the language: what is wanted is the
/// literal a person typed.
///
/// A reader like this fails quietly by default -- a socket written in a shape it
/// does not match drops out of the list, and every check below then passes over
/// a short one, which is precisely the blind spot these cases exist to close.
/// So the two halves are joined here: a socket this reader opened and found no
/// port in panics rather than being skipped.
fn sockets(name: &str) -> Vec<Socket> {
    /// Every socket opened is a socket a port was read out of.
    fn had_a_port(name: &str, open: &Option<String>, found: &[Socket]) {
        let Some(socket) = open else {
            return;
        };
        assert!(
            found.last().is_some_and(|last| last.name == *socket),
            "{name}: {socket} declares no port in a shape this reader matches, so it would drop \
             out of every check over this composition",
        );
    }

    let text = runfile(name);
    let mut found: Vec<Socket> = Vec::new();
    let mut open: Option<String> = None;
    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("udp_socket ") {
            had_a_port(name, &open, &found);
            open = Some(rest.trim().to_string());
        } else if let Some(rest) = line.strip_prefix("port: ") {
            let socket = open
                .clone()
                .unwrap_or_else(|| panic!("{name}: a port outside any socket: {line}"));
            let number = rest
                .trim_end_matches(';')
                .parse()
                .unwrap_or_else(|_| panic!("{name}: {socket} names no port number: {line}"));
            found.push(Socket {
                name: socket,
                port: number,
                incoming: false,
            });
        } else if line == "direction: incoming;" {
            let last = found
                .last_mut()
                .unwrap_or_else(|| panic!("{name}: a direction before any port"));
            last.incoming = true;
        }
    }
    had_a_port(name, &open, &found);
    found
}

#[test]
fn the_control_box_binds_the_ports_the_driver_names() {
    let declared = sockets(ROBOT_CLK);
    assert_eq!(
        declared.len(),
        ports::ALL.len(),
        "one socket per subject of the seam, and no more: {declared:?}",
    );
    let mut numbers: Vec<u16> = declared.iter().map(|socket| socket.port).collect();
    numbers.sort_unstable();
    let mut named = ports::ALL;
    named.sort_unstable();
    assert_eq!(
        numbers,
        named.to_vec(),
        "the numbers the composition types are the numbers the driver binds",
    );
}

#[test]
fn the_injection_port_is_not_a_port_of_the_seam() {
    let injection = sockets(ROBOT_HOST_CLK)
        .into_iter()
        .find(|socket| socket.name == "HostSimCmdIn")
        .expect("the host composition declares the injection socket");
    assert!(
        !ports::ALL.contains(&injection.port),
        "the injection subject exists in no driver, so its number must be no subject's: {injection:?}",
    );
}

#[test]
fn no_two_sockets_of_the_host_composition_bind_one_port() {
    // Both boxes are in it: the control box's own incoming sockets come from
    // the composition it is defined in, and the plant's from the host one.
    let host = sockets(ROBOT_HOST_CLK);
    assert!(
        host.iter().any(|socket| socket.incoming),
        "the host composition binds ports of its own, so a reading of it that finds none is a \
         reading that would pass whatever the file says: {host:?}",
    );
    let mut bound: Vec<Socket> = sockets(ROBOT_CLK)
        .into_iter()
        .chain(host)
        .filter(|socket| socket.incoming)
        .collect();
    bound.sort_by_key(|socket| socket.port);
    for pair in bound.windows(2) {
        assert_ne!(
            pair[0].port, pair[1].port,
            "{} and {} would bind one port on one loopback",
            pair[0].name, pair[1].name,
        );
    }
}
