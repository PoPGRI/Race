# GRAIC 2023
Installation Documentation could be found [Here](https://docs.google.com/document/d/1O0thKd-WcQzPpEvyfJZmjEr0xCWvgUkzzftlyZxOi_A/edit?usp=sharing)

1. All you need to submit is `agent.py`. And all you implementation should be contained in this file.
2. If you want to change to different map, just modify line 6 in `wrapper.py`. 5 maps (shanghai_intl_circuit, t1_triple, t2_triple, t3, t4) have been made available to public, while we hold some hidden maps for testing.
3. If you would like to test your controller without the scenarios, comment out line 10 in `wrapper.py`.

Benchmark score using our very naive controllers on below tracks with **no scenarios**:

| Track | Score|
|-----|--------|
| triple_t1 | 45 |
| triple_t2 | 74 |
| t3 | 82 |
| t4 | 57 |
| shanghai_intl_circuit | 122 |
