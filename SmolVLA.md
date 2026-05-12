# SmolVLA Setup

## Requirements

- Python 3.10
- `lerobot==0.4.0` already installed (check: `pip show lerobot`)

## Installation

```bash
pip install "lerobot[smolvla]==0.4.0"
```

This installs the smolvla extras against the existing lerobot 0.4.0. All major deps (`transformers`, `accelerate`, `safetensors`) are likely already satisfied.

Verify the install:

```bash
python3 -c "from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy; print('OK')"
```

## Finetuning

```bash
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=<HF_USER>/mydataset \
  --batch_size=64 \
  --steps=20000 \
  --output_dir=outputs/train/my_smolvla \
  --policy.device=cuda \
  --wandb.enable=true
```

- Minimum ~50 episodes recommended
- ~4 hours on a single A100 for 20k steps

## Notes

- The lerobot submodule (`third_party/lerobot`) is v0.5.2 and requires Python ≥ 3.12 — **do not** use `pip install -e ".[smolvla]"` from there on this machine (Python 3.10 only).
- Reference: [HuggingFace SmolVLA blog](https://huggingface.co/blog/smolvla)
