# tree-sitter-modelica

An [open-source](OSMC-License.txt) Modelica ([Modelica Language Specification v3.5](https://specification.modelica.org/maint/3.5/MLS.html)) grammar and highlighting-query for [tree-sitter](https://github.com/tree-sitter/tree-sitter).

## Installation

```bash
npm install
npm run build
```

To generate the C code to parse Modelica, run:

```bash
npx tree-sitter generate
# Skip npx if you have ./node_modules/.bin in your PATH
# tree-sitter generate
```

## Unit Tests

There is a number of tests included. To run all tests defined in [test/](./test/), just run:

```bash
npx tree-sitter test
```

### Examples

To test the parser on a Modelica file, you can run:

```bash
npx tree-sitter parse examples/SimpleMath.mo
```

## Highlighting

There is also a highlighting query included. To test the highlighting you can run:

```bash
npx tree-sitter highlight examples/SimpleMath.mo
```

## Current Status

Tree-sitter-modelica has been tested on a "Save Total" version of the DrumBoiler.mo file which was successfully parsed and highlighted.