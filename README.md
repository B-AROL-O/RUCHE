---
title: RUCHE
emoji: 💬
colorFrom: yellow
colorTo: purple
sdk: gradio
sdk_version: 5.49.1
app_file: app.py
pinned: false
hf_oauth: true
hf_oauth_scopes:
  - inference-api
license: mit
short_description: ROS2-based Unified Control for Hugging-face Embodied-agents
---

## RUCHE

Project **RUCHE** (acronym of **R**OS2-based **U**nified **C**ontrol for **H**ugging-face **E**mbodied-agents) is a project developed by the [B-AROL-O Team](https://github.com/B-AROL-O) in response to "[MCP'st 1st Birthday](https://huggingface.co/MCP-1st-Birthday)" Hackathon.
This Hackathon is hosted by [Anthropic](https://www.anthropic.com/) and [Gradio](https://www.gradio.app/) with additional support from [Hugging Face](https://huggingface.co/), [OpenAI](https://openai.com/), [Gemini](https://gemini.google.com/), [Modal](https://modal.com/), [Sambanova](https://sambanova.ai/), [IIElevenLabs](https://elevenlabs.io/), [Blaxel](https://blaxel.ai/), [Llamaindex](https://www.llamaindex.ai/) and [Nebius](https://nebius.com/).

Incidentally, since [Ruché](https://en.wikipedia.org/wiki/Ruch%C3%A9) is a red [Italian wine](https://en.wikipedia.org/wiki/Italian_wine) [grape variety](https://en.wikipedia.org/wiki/Grape_variety) from the [Piedmont](<https://en.wikipedia.org/wiki/Piedmont_(wine)>) region, the name RUCHE was chosen to keep up with the naming convention of [B-AROL-O Team](https://github.com/B-AROL-O) Open Source projects, following [ARNEIS](https://github.com/B-AROL-O/ARNEIS), [FREISA](https://github.com/B-AROL-O/FREISA), etc.

<!-- TODO: Add a brief description of the project here. -->

<!-- From <https://huggingface.co/spaces/gradio-templates/chatbot>
An example chatbot using [Gradio](https://gradio.app), [`huggingface_hub`](https://huggingface.co/docs/huggingface_hub/v0.22.2/en/index), and the [Hugging Face Inference API](https://huggingface.co/docs/api-inference/index).
-->

## How to run the RUCHE project

The RUCHE project can easily be run using a [Development Container](https://containers.dev/).

### Inside the Hugging Face Space

The `main` branch of the RUCHE project is automatically synchronized to space <https://huggingface.co/spaces/MCP-1st-Birthday/RUCHE>, therefore to run the project you can simply open this URL inside a web browser

<!-- TODO: Add screenshot -->

- Click **Sign in with Hugging Face**, then enter your message and wait for the results

### Using GitHub Codespaces

You can use [GitHub Codespaces](https://github.com/features/codespaces) to create a Development Environment for the RUCHE project:

- From your browser open <https://github.com/B-AROL-O/RUCHE>
- Choose either the `main` (production) branch or other branches / Pull Requests you want to test
- Click the **\<\> Code** button, then in tab "Codespaces" click **+** to create a new codespace
- Open a bash Terminal and type:

  ```bash
  # Login to Gradio
  uv run hf auth login

  # Run the RUCHE chatbot app
  uv run app.py
  ```

  <!-- TODO: Add screenshot -->

- Continue as described in section "Inside the Hugging Face Space"

### Using Visual Studio Code

Alternatively, if you have a laptop (Windows, macOS or Linux) with the following installed software:

- [Visual Studio Code](https://code.visualstudio.com/) with the following extensions: [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- A recent release of [Docker](https://www.docker.com/) (either [Docker Engine](https://docs.docker.com/engine/) or [Docker Desktop](https://www.docker.com/products/docker-desktop/))

you can create a local Development Environment with the following steps:

- Launch Visual Studio
- Open the Command Palette and type:
  > Git: Clone
  > then specify the repository <https://github.com/B-AROL-O/RUCHE>
- Open the Command Palette and type:
  > Dev Containers: Rebuild and Reopen in Container
- Open a bash Terminal and type:

  ```bash
  # Login to Gradio
  uv run hf auth login

  # Run the RUCHE chatbot app
  uv run app.py
  ```

- Read the instructions displayed in the terminal and open the link in your browser

  ![2025-11-17-vscode-run-app04.png](docs/images/2025-11-17-vscode-run-app04.png)

- Continue as described in section "Inside the Hugging Face Space"

## Copyright and License

Copyright (C) 2025, [B-AROL-O Team](https://github.com/B-AROL-O), all rights reserved.

**NOTE**: This repository is based upon <https://github.com/arol-polito/python-project-template>.

### Source code license

The source code contained in this repository and the executable distributions are licensed under the terms of the MIT license as detailed in the [LICENSE](LICENSE) file.

### Documentation license

![CC BY-SA 4.0](https://i.creativecommons.org/l/by-sa/4.0/88x31.png)

Please note that your contribution to the project Documentation is licensed under a Creative Commons Attribution-Share Alike 4.0 License. see <https://creativecommons.org/licenses/by-sa/4.0/>

<!-- EOF -->
