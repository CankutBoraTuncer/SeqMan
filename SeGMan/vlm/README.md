# Requirements

Install the requirements using:

```bash
pip install -r requirements.txt
```

Also, open an empty `/tmp` directory under `/vlm` to store images that are generated for overlaying grids on top of configurations.

# Instructions

Run the prompt test by navigating to `/vlm` and running prompt_test.py via:

```bash
cd SeGMan/vlm
python prompt_test.py <CONFIG_NAME>
```

Where `<CONFIG_NAME>` is the name of the configuration file you want to test. The default is `p1-two-blocks`.

---

There are two prompts used in the test both of which are available in `prompt_test.py`:
- SYSTEM_PROMPT: A long and descriptive prompt that gives context and instructions to the model.
- prompt (generated in `build_prompt()`): A prompt with config-specific information. Currently, it contains only the bare-minimum information required to solve the puzzles.

These prompts can be modified, improved, and tested to approach better results.

# Output

The output will contain responses for 4 overlay versions of the same input configuration because we don't know yet which overlay makes the model perform best. The responses are `VLMResponse` instances defined in `prompt_test.py` using OpenAI's structured outputs. If the model returns a non-empty obstacle-position list, then the response will be visualized with a scene view too.