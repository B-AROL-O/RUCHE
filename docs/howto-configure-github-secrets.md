## HOWTO Configure GitHub Secrets

## Introduction

This document explains how to properly configure GitHub Secrets and Variables in order to properly deploy the RUCHE project using GitHub Workflows.

## Step-by-step instructions

### Create an Access Token in Hugging Face

Open <https://huggingface.co/settings/tokens>)

- Click **Create new token**

> **Create new Access Token**

- Token tpe: **Write** <!-- Fine-grained | Read | Write -->
- Token name: `TODO`

<!--
- User permissions (gmacario): `TODO`
-->

then click **Create token**.

<!-- <img width="1138" height="702" alt="image" src="https://gist.github.com/user-attachments/assets/66760edf-d4ce-45cf-a281-2b817eb5c98b" /> -->

![2025-11-20-configure-github-secrets01.png](images/2025-11-20-configure-github-secrets01.png)

Click **Copy** the value of the HF Access Token and save it in a safe place.

## Configure Repository Secrets and Variables

Open <https://github.com/B-AROL-O/RUCHE>

- Click on **Settings**
- In section "Security", click on **Secrets and variables** > **Actions**

<!-- <img width="1271" height="1009" alt="image" src="https://gist.github.com/user-attachments/assets/d3d0e88b-ed90-43e3-ad9a-d5d8ff16024b" /> -->

![2025-11-20-configure-github-secrets02.png](images/2025-11-20-configure-github-secrets02.png)

Select tab "Repository secrets", then click on **New repository secret**

- **Name \***: `HF_TOKEN`
- **Secret \***: `hf_xxxx` (paste the value of the HF Access Token you created earlier)
- Click **Add secret**

Result:

<!-- <img width="1127" height="991" alt="image" src="https://gist.github.com/user-attachments/assets/04f82106-2bcb-47a5-b37f-794273d9b410" /> -->

![2025-11-20-configure-github-secrets03.png](images/2025-11-20-configure-github-secrets03.png)

Select tab "Repository variables", then click on **New repository variable**

- **Name \***: `HF_USERNAME`
- **Value \***: (your Hugging Face username, example: `gmacario`)
- Click **Add variable**

Always in tab "Repository variables", click on **New repository variable**

- **Name \***: `HF_ORGNAME`
- **Value \***: (the HF organization where the space belongs, example: `MCP-1st-Birthday`)
- Click **Add variable**

Always in tab "Repository variables", click on **New repository variable**

- **Name \***: `HF_SPACENAME`
- **Value \***: (the name of the HF space, example: `RUCHE`)
- Click **Add variable**

Result:

<!-- <img width="1127" height="997" alt="image" src="https://gist.github.com/user-attachments/assets/699b9d77-7a7f-44ec-bf37-1d1e5e85e85e" /> -->

![2025-11-20-configure-github-secrets04.png](images/2025-11-20-configure-github-secrets04.png)

<!-- EOF -->
