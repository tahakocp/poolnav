use navcore::Config;
use poolsim::{pool, variant, Params, World};
use std::sync::mpsc;
use std::thread;

const USAGE: &str = "usage:
  poolsim bench [--seeds N] [--minutes M] [--pool NAME]... [--variant NAME]...
                [--gyro-bias DPS] [--speed-err FRACTION] [--align DPS]
  poolsim curve <pool> <variant> [seeds] [minutes]   median coverage per minute
  poolsim trace <pool> <variant> <seed> [minutes]    path as CSV";

#[derive(Default, Clone)]
struct Outcome {
    t90: Option<f64>,
    t95: Option<f64>,
    t99: Option<f64>,
    final_cov: f64,
    swept_ratio: f64,
}

fn run_one(poolname: &str, cfg: Config, seed: u64, params: &Params, minutes: f64) -> Outcome {
    let mut w = World::new(pool::by_name(poolname).unwrap(), cfg, seed, params.clone());
    let mut o = Outcome::default();
    while w.t < minutes * 60.0 {
        w.step();
        let c = w.coverage();
        if o.t90.is_none() && c >= 0.90 {
            o.t90 = Some(w.t);
        }
        if o.t95.is_none() && c >= 0.95 {
            o.t95 = Some(w.t);
        }
        if o.t99.is_none() && c >= 0.99 {
            o.t99 = Some(w.t);
            break;
        }
    }
    o.final_cov = w.coverage();
    o.swept_ratio = w.overlap();
    o
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    v[v.len() / 2]
}

fn bench(args: &[String]) {
    let mut seeds = 100u64;
    let mut minutes = 120.0;
    let mut pools: Vec<String> = vec![];
    let mut variants: Vec<String> = vec![];
    let mut params = Params::default();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--seeds" => seeds = args[i + 1].parse().unwrap(),
            "--minutes" => minutes = args[i + 1].parse().unwrap(),
            "--pool" => pools.push(args[i + 1].clone()),
            "--variant" => variants.push(args[i + 1].clone()),
            "--gyro-bias" => params.gyro_bias_dps = args[i + 1].parse().unwrap(),
            "--speed-err" => params.speed_err = args[i + 1].parse().unwrap(),
            "--align" => params.align_dps = args[i + 1].parse().unwrap(),
            _ => {
                eprintln!("{USAGE}");
                std::process::exit(2);
            }
        }
        i += 2;
    }
    if pools.is_empty() {
        pools = pool::NAMES.iter().map(|s| s.to_string()).collect();
    }
    if variants.is_empty() {
        variants = ["random", "lanes"].iter().map(|s| s.to_string()).collect();
    }
    println!("pool,variant,runs,median_min_to_90,median_min_to_95,reached_99_pct,median_min_to_99,median_final_cov_pct,median_swept_over_cleaned");
    for p in &pools {
        for v in &variants {
            let cfg = variant(v).unwrap_or_else(|| panic!("unknown variant {v}"));
            let (tx, rx) = mpsc::channel();
            let workers = thread::available_parallelism()
                .map(|n| n.get())
                .unwrap_or(4) as u64;
            let mut hs = vec![];
            for k in 0..workers {
                let (tx, p, params) = (tx.clone(), p.clone(), params.clone());
                hs.push(thread::spawn(move || {
                    let mut s = k;
                    while s < seeds {
                        tx.send(run_one(&p, cfg, s + 1, &params, minutes)).unwrap();
                        s += workers;
                    }
                }));
            }
            drop(tx);
            let outs: Vec<Outcome> = rx.iter().collect();
            for h in hs {
                h.join().unwrap();
            }
            let cap = minutes * 60.0;
            let m = |f: fn(&Outcome) -> Option<f64>| {
                median(outs.iter().map(|o| f(o).unwrap_or(cap * 2.0)).collect()) / 60.0
            };
            let m90 = m(|o| o.t90);
            let m95 = m(|o| o.t95);
            let m99 = m(|o| o.t99);
            let r99 = outs.iter().filter(|o| o.t99.is_some()).count() * 100 / outs.len();
            let fin = median(outs.iter().map(|o| o.final_cov).collect()) * 100.0;
            let sw = median(outs.iter().map(|o| o.swept_ratio).collect());
            let show = |x: f64| {
                if x > minutes {
                    format!(">{minutes:.0}")
                } else {
                    format!("{x:.1}")
                }
            };
            println!(
                "{p},{v},{},{},{},{r99},{},{fin:.1},{sw:.2}",
                outs.len(),
                show(m90),
                show(m95),
                show(m99)
            );
        }
    }
}

fn curve(args: &[String]) {
    let (poolname, var) = (&args[0], &args[1]);
    let seeds: u64 = args.get(2).map(|s| s.parse().unwrap()).unwrap_or(100);
    let minutes: usize = args.get(3).map(|s| s.parse().unwrap()).unwrap_or(60);
    let cfg = variant(var).expect("variant");
    let mut all: Vec<Vec<f64>> = vec![];
    for seed in 1..=seeds {
        let mut w = World::new(
            pool::by_name(poolname).expect("pool"),
            cfg,
            seed,
            Params::default(),
        );
        let mut c = vec![0.0];
        for _ in 0..minutes {
            for _ in 0..600 {
                w.step();
            }
            c.push(w.coverage());
        }
        all.push(c);
    }
    println!("minute,p10,median,p90");
    for m in 0..=minutes {
        let mut v: Vec<f64> = all.iter().map(|c| c[m]).collect();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let at = |q: f64| v[((v.len() - 1) as f64 * q).round() as usize];
        println!("{m},{:.4},{:.4},{:.4}", at(0.1), at(0.5), at(0.9));
    }
}

fn trace(args: &[String]) {
    let (poolname, var, seed) = (&args[0], &args[1], args[2].parse::<u64>().unwrap());
    let minutes: f64 = args.get(3).map(|s| s.parse().unwrap()).unwrap_or(30.0);
    let cfg = variant(var).expect("variant");
    let mut w = World::new(
        pool::by_name(poolname).expect("pool"),
        cfg,
        seed,
        Params::default(),
    );
    let mut n = 0u64;
    println!("t,x,y,th,coverage,state");
    while w.t < minutes * 60.0 {
        w.step();
        n += 1;
        if n % 5 == 0 {
            let st = format!("{:?}", w.nav.state());
            let st = st.split(['(', ' ']).next().unwrap_or("");
            println!(
                "{:.1},{:.3},{:.3},{:.3},{:.4},{}",
                w.t,
                w.x,
                w.y,
                w.th,
                w.coverage(),
                st
            );
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("bench") => bench(&args[1..]),
        Some("curve") => curve(&args[1..]),
        Some("trace") => trace(&args[1..]),
        _ => {
            eprintln!("{USAGE}");
            std::process::exit(2);
        }
    }
}
