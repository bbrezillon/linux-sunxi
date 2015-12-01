/// Fix NAND controller drivers declaring their own mtd_info struct instead
/// of the one provided in struct nand_chip.
///
// Confidence: Low
// Copyright: (C) 2015 Boris Brezillon GPL v2.

virtual patch
virtual context
virtual org
virtual report

@match1@
identifier __chipfield, __mtdfield;
type __type;
@@
(
	__type {
		...
		struct nand_chip __chipfield;
		...
		struct mtd_info __mtdfield;
		...
	};
|
	__type {
		...
		struct mtd_info __mtdfield;
		...
		struct nand_chip __chipfield;
		...
	};
)

@fix1 depends on match1 && patch && !context && !org && !report@
identifier match1.__mtdfield;
type match1.__type;
@@
	__type {
		...
-		struct mtd_info __mtdfield;
		...
	};

@fix2 depends on match1 && patch && !context && !org && !report@
identifier match1.__chipfield, match1.__mtdfield;
identifier __subfield;
type match1.__type;
__type *__priv;
@@
<...
(
-	__priv->__mtdfield.__subfield
+	nand_to_mtd(&__priv->__chipfield)->__subfield
|
-	&(__priv->__mtdfield)
+	nand_to_mtd(&__priv->__chipfield)
)
...>

// ----------------------------------------------------------------------------

@fix1_context depends on match1 && !patch && (context || org || report)@
identifier match1.__mtdfield;
type match1.__type;
position j0;
@@

	__type {
		...
*		struct mtd_info __mtdfield@j0;
		...
	};

// ----------------------------------------------------------------------------

@script:python fix1_org depends on org@
j0 << fix1_context.j0;
@@

msg = "struct nand_chip already embeds an mtd_info object (use nand_to_mtd())."
coccilib.org.print_todo(j0[0], msg)

// ----------------------------------------------------------------------------

@script:python fix1_report depends on report@
j0 << fix1_context.j0;
@@

msg = "struct nand_chip already embeds an mtd_info object (use nand_to_mtd())."
coccilib.report.print_report(j0[0], msg)

