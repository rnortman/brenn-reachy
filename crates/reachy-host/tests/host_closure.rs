//! What the voice host's third-party closure actually links.
//!
//! The two native libraries under `reachy-host` -- OpenSSL, for the pod link's
//! TLS-PSK and the bus dial, and ONNX Runtime, for the wake and VAD models --
//! are supplied by the build rather than found on the machine: their crates'
//! discovery build scripts are switched off and the answers those scripts would
//! have produced are stated in `MODULE.bazel`. A stated answer is a claim about
//! a binary nobody has asked, and a wrong one is an ABI mismatch that compiles.
//!
//! So this asks. Every case here calls into the linked library and compares
//! what it says about itself with the version the build pinned. It is also the
//! only place the closure is linked into an executable at all, which is what
//! makes it the gate's proof that `-lssl`, `-lcrypto` and `-lonnxruntime`
//! resolve for the platform being built.

/// The OpenSSL the module graph pins. `ossl330` and the rest of the version
/// `--cfg` flags in `MODULE.bazel` are derived from exactly this number.
const OPENSSL_VERSION: &str = "3.3.1";

/// The commit the ONNX Runtime release fetched in `MODULE.bazel` was cut from,
/// as its own `GIT_COMMIT_ID` file states it. The build info string the runtime
/// answers with carries the commit but not the release number, so this is the
/// identity the linked library can actually be asked for.
const ONNXRUNTIME_COMMIT: &str = "058787ceea";

#[test]
fn openssl_is_the_pinned_version() {
    let reported = openssl::version::version();
    assert!(
        reported.starts_with(&format!("OpenSSL {OPENSSL_VERSION} ")),
        "linked OpenSSL says {reported:?}, expected {OPENSSL_VERSION}"
    );
}

#[test]
fn openssl_carries_the_ciphers_the_pod_link_needs() {
    // The pod link is TLS-PSK, so a build of OpenSSL with PSK compiled out
    // would link and then fail to negotiate on the first connection. The cfg
    // flags name only `OPENSSL_NO_SSL3_METHOD`; this is the same claim asked of
    // the library -- and asked through the function that pins the link's
    // parameters on the wire, so what is proved available is the one suite and
    // the TLS 1.2 pinning the pod actually negotiates, not the `PSK` group
    // alias a plain-PSK-only build would also answer to.
    let mut ctx = openssl::ssl::SslContext::builder(openssl::ssl::SslMethod::tls()).unwrap();
    speech_surface::psk::pin_link_params(&mut ctx).expect("the pod link's TLS parameters");
}

#[test]
fn onnxruntime_is_the_pinned_build() {
    // `info()` is the first call into the runtime, so it is also where a
    // missing or unloadable `libonnxruntime` shows up.
    let reported = ort::info();
    assert!(
        reported.contains(ONNXRUNTIME_COMMIT),
        "linked ONNX Runtime says {reported:?}, expected the build at {ONNXRUNTIME_COMMIT}"
    );
}

#[test]
fn the_linked_runtime_answers_at_the_api_level_ort_asks_for() {
    // `ort` is compiled against API level `ort::MINOR_VERSION` and asks the
    // linked `OrtGetApiBase` for exactly that table; a runtime built below that
    // level answers null there and this call dies. That is the ABI mismatch
    // that compiles, and the only place it is visible is the library itself --
    // comparing the constant against another constant would restate the
    // lockfile. The assertion is incidental (a reference is never null); the
    // call is the case.
    let api: *const _ = ort::api();
    assert!(
        !api.is_null(),
        "the linked ONNX Runtime answered no API table at level {}",
        ort::MINOR_VERSION
    );
}

#[test]
fn the_pipeline_libraries_link() {
    // Naming a type from each brenn-pod crate the host composes is what puts
    // their rlibs -- and with them every native library they carry -- into this
    // executable's link. Three crates, three names: the surface the host runs,
    // the pipeline stages it drives, and the bus attachment it holds. The
    // assertions are incidental; the link is the case.
    let out = speech_surface::ScriptOut {
        pod: String::from("kitchen-reachy"),
        seq: 1,
        body: String::from("{}"),
    };
    assert_eq!(out.seq, 1);

    let pod = speech_pipeline::PodId(out.pod);
    assert_eq!(pod.0, "kitchen-reachy");

    assert_eq!(brenn_bridge::Urgency::Normal.as_str(), "normal");
}

#[test]
fn the_alert_severity_the_surface_names_is_the_attachment_s_own() {
    // `speech_surface::AlertSeverity` must be `brenn_bridge::AlertSeverity` —
    // the same type, not a structurally similar copy. If the surface defined
    // its own enum the host would still compile but silently carry two
    // unrelated severity types. This is the only executable linking both
    // crates. The binding is the case; it fails to compile, not at runtime.
    let _: speech_surface::AlertSeverity = brenn_bridge::AlertSeverity::Warning;
}
