# HOWTO Configure GitHub Secrets

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

![2025-11-20-configure-github-secrets01.png](images/2025-11-20-configure-github-secrets01.png)

Click **Copy** the value of the HF Access Token and save it in a safe place.

## Configure Repository Secrets and Variables

Open <https://github.com/B-AROL-O/RUCHE>

- Click on **Settings**
- In section "Security", click on **Secrets and variables** > **Actions**

![2025-11-20-configure-github-secrets02.png](images/2025-11-20-configure-github-secrets02.png)

Select tab "Repository secrets", then click on **New repository secret**

- **Name \***: `HF_TOKEN`
- **Secret \***: `hf_xxxx` (paste the value of the HF Access Token you created earlier)
- Click **Add secret**

Result:

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

![2025-11-20-configure-github-secrets04.png](images/2025-11-20-configure-github-secrets04.png)

<!-- EOF -->
