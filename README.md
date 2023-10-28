[![build-parser](https://github.com/OpenModelica/tree-sitter-modelica/actions/workflows/build-parser.yml/badge.svg)](https://github.com/OpenModelica/tree-sitter-modelica/actions/workflows/build-parser.yml)

# tree-sitter-modelica

An [open-source](OSMC-License.txt) Modelica
([Modelica Language Specification v3.5](https://specification.modelica.org/maint/3.5/MLS.html))
grammar and highlighting-query for
[tree-sitter](https://github.com/tree-sitter/tree-sitter).

## Dependencies

  - Node.js
  - Docker

## Installation

```bash
npm install
npm run build
```

To generate the C code to parse Modelica run:

```bash
npx tree-sitter generate
```

> [!NOTE]
> If you have ./node_modules/.bin in your `PATH` environment variable you can skip `npx`
> ```bash
> tree-sitter generate
> ```

## Unit Tests

There is a number of tests included. To run all tests defined in [test/](./test/) just run:

```bash
npx tree-sitter test
```

### Examples

To test the parser on a Modelica file run:

```bash
npx tree-sitter parse examples/SimpleMath.mo
```

## Highlighting

There is also a highlighting query included.
Make sure that the
[tree-sitter per-user configuration](https://tree-sitter.github.io/tree-sitter/syntax-highlighting#per-user-configuration)
are pointing to the parent directory of `tree-sitter-modelica`.
So if this directory is in `/home/USER/workspace/tree-sitter-modelica` add
`/home/USER/workspace` to the parser directories:

**config.json**
```json
{
  "parser-directories": [
    "/home/USER/workspace"
  ],
}
```

To test the highlighting configure run:

```bash
npx tree-sitter highlight examples/SimpleMath.mo
```

## Usage

Use [Web Tree-sitter](https://github.com/tree-sitter/tree-sitter/blob/master/lib/binding_web/README.md)
`tree-sitter-modelica.wasm` in your application:

```typescript
import * as Parser from 'web-tree-sitter'

await Parser.init()
const parser = new Parser

const Modelica = await Parser.Language.load(`tree-sitter-modelica.wasm`)
parser.setLanguage(Modelica)
```

## Current Status

Tree-sitter-modelica has been tested on a "Save Total" version of the
[Modelica.Fluid.Examples.DrumBoiler.DrumBoiler](./examples/DrumBoiler.mo) which was
successfully parsed and highlighted.

```bash
npx tree-sitter parse examples/DrumBoiler.mo
npx tree-sitter highlight examples/DrumBoiler.mo
```
